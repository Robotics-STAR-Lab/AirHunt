import os
import numpy as np
import logging
import random
import torch
import time
import cv2
import airsim
from PIL import Image
from groundingdino.datasets import transforms as dino_transforms
import supervision as sv
from scipy.spatial.transform import Rotation as R
from collections import Counter
from typing import List, Optional, Tuple, Dict
import copy
from groundingdino.util.inference import predict
from torchvision.ops import box_convert
from airhunt.geometry import get_cam_intr
from airhunt.memory import SnapShot
from airhunt.clustering import SceneHierarchicalClustering
from airhunt.mapping.utils.ious import mask_subtract_contained
from airhunt.mapping.utils.general_utils import (
    ObjectClasses,
    measure_time,
    filter_detections,
)
from airhunt.mapping.slam.slam_classes import MapObjectDict, DetectionDict, to_tensor
from airhunt.mapping.slam.utils import (
    filter_gobs,
    filter_objects,
    get_bounding_box,
    init_process_pcd,
    denoise_objects,
    merge_objects,
    detections_to_obj_pcd_and_bbox,
    processing_needed,
    resize_gobs,
    merge_obj2_into_obj1,
)
from airhunt.mapping.slam.mapping import (
    compute_spatial_similarities,
    compute_visual_similarities,
    aggregate_similarities,
    match_detections_to_objects,
)
from airhunt.mapping.utils.model_utils import compute_clip_features_batched
from airhunt.bridge import AirsimBridge


def pos_memory_to_airsim(pts):
    return np.array([-pts[2], pts[0], -pts[1]])


class Scene:

    def __init__(
        self,
        scene_id,
        env_name,
        cfg,
        graph_cfg,
        detection_model,
        sam_predictor,
        clip_model,
        clip_preprocess,
        clip_tokenizer,
        init_pts_memory,
        logging_path,
        formatter,
    ):
        self.cfg = cfg
        self.cfg_cg = graph_cfg
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.scene_id = scene_id
        self.airsim_bridge = AirsimBridge(env_name, self.cfg, logging_path, formatter)
        self.pathfinder = AirsimPathfinder(self.airsim_bridge, init_pts_memory)
        self.cam_intrinsic = get_cam_intr(cfg.hfov, cfg.img_height, cfg.img_width)
        self.obj_classes = AirsimObjectClasses(
            bg_classes=self.cfg_cg["bg_classes"],
            skip_bg=self.cfg_cg["skip_bg"],
            class_set=self.cfg["class_set"],
        )
        logging.info(f"Load AirSim scene {scene_id} successfully")
        self.objects: MapObjectDict[int, Dict] = MapObjectDict()
        self.object_id_counter = 1
        self.snapshots: Dict[str, SnapShot] = {}
        self.frames: Dict[str, SnapShot] = {}
        self.all_observations: Dict[str, np.ndarray] = {}
        self.clustering = SceneHierarchicalClustering(
            min_sample_split=0, random_state=66
        )
        self.detection_model = detection_model
        self.sam_predictor = sam_predictor
        self.clip_model = clip_model.to(self.device)
        self.clip_preprocess = clip_preprocess
        self.clip_tokenizer = clip_tokenizer

    def clear_up_detections(self):
        self.objects = MapObjectDict()
        self.object_id_counter = 1
        self.snapshots = {}
        self.frames = {}
        self.all_observations = {}

    def get_observation_bk(self, pts, angle=None, rotation=None, is_frontier=False):
        assert (angle is None) ^ (
            rotation is None
        ), "Only one of angle and rotation should be specified"
        if angle is not None:
            pitch, yaw, roll = (0.0, angle, 0.0)
        else:
            r = R.from_quat([rotation[0], rotation[1], rotation[2], rotation[3]])
            pitch, yaw, roll = r.as_euler("xyz")
        if self.cfg.get("ros_planner", False):
            self.airsim_bridge.set_drone_pos_ros(
                pts[0], pts[1], pts[2], pitch, yaw, roll, is_frontier
            )
        else:
            self.airsim_bridge.set_drone_pos(pts[0], pts[1], pts[2], pitch, yaw, roll)
        rgb, depth = self.airsim_bridge.get_camera_data()
        obs = {"color_sensor": rgb, "depth_sensor": depth, "semantic_sensor": None}
        airsim_pos, airsim_ori = self.airsim_bridge.get_pose()
        pitch, roll, yaw = airsim.to_eularian_angles(airsim_ori)
        R_airsim = R.from_euler("xyz", [pitch, roll, yaw]).as_matrix()
        t_airsim = airsim_pos
        pose_airsim = np.eye(4)
        pose_airsim[:3, :3] = R_airsim
        pose_airsim[:3, 3] = t_airsim
        T = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])
        T_4x4 = np.eye(4)
        T_4x4[:3, :3] = T
        cam_pose = T_4x4 @ pose_airsim @ T_4x4.T
        return (obs, cam_pose, airsim_pos, (pitch, roll, yaw))

    def get_observation(self, pts, roll=0.0, pitch=0.0, yaw=0.0, is_frontier=False):
        if self.cfg.get("ros_planner", False):
            self.airsim_bridge.set_drone_pos_ros(
                pts[0], pts[1], pts[2], pitch, yaw, roll, is_frontier
            )
        else:
            self.airsim_bridge.set_drone_pos(pts[0], pts[1], pts[2], pitch, yaw, roll)
        time.sleep(1.0)
        all_camera_data = self.airsim_bridge.get_all_camera_data_from_ros(
            ["front", "left", "right"]
        )
        logging.info("get camera data")
        print(
            f"Target angles (degrees): Yaw: {np.degrees(yaw):.2f}, Roll: {np.degrees(roll):.2f}, Pitch: {np.degrees(pitch):.2f}\n"
        )
        processed_cameras = {}
        processed_cameras = {}
        for camera_name, camera_data in all_camera_data.items():
            cam_pose_memory, cam_pose_ned = self.convert_pose_msg_to_memory(
                camera_data["pose"]
            )
            processed_cameras[camera_name] = {
                "color": camera_data["rgb"],
                "depth": camera_data["depth"],
                "cam_pose_memory": cam_pose_memory,
                "cam_pose_ned": cam_pose_ned,
            }
        drone_pos, drone_ori = self.airsim_bridge.get_pose()
        drone_pitch, drone_roll, drone_yaw = airsim.to_eularian_angles(drone_ori)
        if not np.allclose(drone_pos, pts):
            raise ValueError("set drone pos fail")
        return (processed_cameras, drone_pos, (drone_pitch, drone_roll, drone_yaw))

    def get_frontier_observation(self, pts, view_dir, camera_tilt=0.0):
        view_dir_airsim = np.array([-view_dir[2], view_dir[0], 0.0])
        default_view_dir_airsim = np.asarray([1.0, 0.0, 0.0])
        if np.linalg.norm(view_dir_airsim) < 0.001:
            view_dir_airsim = default_view_dir_airsim
        view_dir_airsim = view_dir_airsim / np.linalg.norm(view_dir_airsim)
        yaw = np.arctan2(view_dir_airsim[1], view_dir_airsim[0])
        if (
            np.dot(view_dir_airsim, default_view_dir_airsim)
            / np.linalg.norm(view_dir_airsim)
            < -1 + 0.001
        ):
            yaw = np.pi
        pts_airsim = pos_memory_to_airsim(pts)
        obs = None
        yaw_memory = -yaw % (2 * np.pi)
        return (obs, yaw_memory)

    def update_scene_graph(
        self,
        image_rgb: np.ndarray,
        depth: np.ndarray,
        intrinsics,
        cam_pose,
        pts,
        angle: float,
        img_path,
        text_prompt: str = None,
    ):
        start_time = time.time()
        obj_classes = self.obj_classes
        if text_prompt is None:
            text_prompt = ". ".join(obj_classes.get_classes_arr()) + "."
        if not text_prompt.endswith("."):
            text_prompt += "."
        self.sam_predictor.set_image(image_rgb)
        dino_transform = dino_transforms.Compose(
            [
                dino_transforms.RandomResize([800], max_size=1333),
                dino_transforms.ToTensor(),
                dino_transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        image_tensor, _ = dino_transform(Image.fromarray(image_rgb), None)
        with torch.cuda.amp.autocast(enabled=False):
            boxes, confidences, labels = predict(
                model=self.detection_model,
                image=image_tensor,
                caption=text_prompt,
                box_threshold=self.cfg.box_threshold,
                text_threshold=self.cfg.text_threshold,
                device=self.device,
            )
        from airhunt.labels import canonical_object_label

        canonical_labels = [
            canonical_object_label(label, obj_classes.get_classes_arr())
            for label in labels
        ]
        valid_indices = [
            i for i, label in enumerate(canonical_labels) if label is not None
        ]
        for raw_label, canonical in zip(labels, canonical_labels):
            if canonical is None and raw_label.strip():
                logging.warning("Skipping unsupported detector label: %r", raw_label)
        labels = [canonical_labels[i] for i in valid_indices]
        print(labels)
        boxes = boxes[valid_indices]
        confidences = confidences[valid_indices]
        if len(boxes) == 0:
            xyxy_np = np.empty((0, 4), dtype=np.float32)
            masks_np = np.empty((0, *image_rgb.shape[:2]), dtype=np.float64)
            confidences = np.empty((0,), dtype=np.float32)
            detection_class_ids = np.empty((0,), dtype=int)
            detection_class_labels = []
        else:
            h, w = image_rgb.shape[:2]
            boxes = boxes * torch.Tensor([w, h, w, h])
            input_boxes = box_convert(
                boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy"
            ).numpy()
            xyxy_np = input_boxes
            if hasattr(confidences, "numpy"):
                confidences = confidences.numpy()
            elif hasattr(confidences, "cpu"):
                confidences = confidences.cpu().numpy()
            confidences = confidences.astype(np.float32)
            detection_class_ids = []
            classes_arr = obj_classes.get_classes_arr()
            for label in labels:
                class_id = classes_arr.index(label)
                detection_class_ids.append(class_id)
            detection_class_ids = np.array(detection_class_ids, dtype=int)
            detection_class_labels = labels
            masks, scores, logits = self.sam_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=input_boxes,
                multimask_output=False,
            )
            if masks.ndim == 4:
                masks = masks.squeeze(1)
            masks_np = masks.astype(np.float64)
            detection_output_dir = "outputs/grounded_sam2_detections"
            os.makedirs(detection_output_dir, exist_ok=True)
            class_ids = np.array(list(range(len(labels))))
            labels_with_conf = [
                f"{class_name} {confidence:.2f}"
                for class_name, confidence in zip(labels, confidences)
            ]
            detections = sv.Detections(
                xyxy=input_boxes, mask=masks_np.astype(bool), class_id=class_ids
            )
            annotated_frame = image_rgb.copy()
            box_annotator = sv.BoxAnnotator()
            annotated_frame = box_annotator.annotate(
                scene=annotated_frame, detections=detections
            )
            label_annotator = sv.LabelAnnotator()
            annotated_frame = label_annotator.annotate(
                scene=annotated_frame, detections=detections, labels=labels_with_conf
            )
            safe_prompt = (
                text_prompt.replace(".", "").replace(" ", "_")
                if text_prompt
                else "detection"
            )
            bbox_filename = f"{safe_prompt}_{img_path.replace('/', '_').replace('.png', '')}_bbox.jpg"
            cv2.imwrite(
                os.path.join(detection_output_dir, bbox_filename),
                cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR),
            )
            mask_annotator = sv.MaskAnnotator()
            annotated_frame = mask_annotator.annotate(
                scene=annotated_frame, detections=detections
            )
            mask_filename = f"{safe_prompt}_{img_path.replace('/', '_').replace('.png', '')}_mask.jpg"
            cv2.imwrite(
                os.path.join(detection_output_dir, mask_filename),
                cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR),
            )
            logging.info(f"Saved detection images: {bbox_filename}, {mask_filename}")
        end_time = time.time()
        duration = end_time - start_time
        logging.info(f"update_scene_graph execution time: {duration:.4f} seconds")
        curr_det = sv.Detections(
            xyxy=xyxy_np,
            confidence=confidences,
            class_id=detection_class_ids,
            mask=masks_np,
        )
        if len(curr_det) == 0:
            logging.debug("No detections in this frame")
            end_time = time.time()
            print(
                f"update_scene_graph execution time: {end_time - start_time:.4f} seconds"
            )
            return (image_rgb, [])
        curr_det, labels = filter_detections(
            image=image_rgb,
            detections=curr_det,
            classes=obj_classes,
            given_labels=detection_class_labels,
            iou_threshold=self.cfg_cg.object_detection_iou_threshold,
            min_mask_size_ratio=self.cfg_cg.min_mask_size_ratio,
            confidence_threshold=self.cfg_cg.object_detection_confidence_threshold,
        )
        if curr_det is None:
            logging.debug("No detections left after filter_detections")
            end_time = time.time()
            print(
                f"update_scene_graph execution time: {end_time - start_time:.4f} seconds"
            )
            return (image_rgb, [])
        image_crops, image_feats, text_feats = compute_clip_features_batched(
            image_rgb,
            curr_det,
            self.clip_model,
            self.clip_preprocess,
            self.clip_tokenizer,
            obj_classes.get_classes_arr(),
            self.device,
        )
        raw_gobs = {
            "xyxy": curr_det.xyxy,
            "confidence": curr_det.confidence,
            "class_id": curr_det.class_id,
            "mask": curr_det.mask,
            "classes": obj_classes.get_classes_arr(),
            "image_crops": image_crops,
            "image_feats": image_feats,
            "text_feats": text_feats,
            "detection_class_labels": detection_class_labels,
        }
        resized_gobs = resize_gobs(raw_gobs, image_rgb)
        filtered_gobs = filter_gobs(
            resized_gobs,
            image_rgb,
            skip_bg=self.cfg_cg.skip_bg,
            BG_CLASSES=obj_classes.get_bg_classes_arr(),
            mask_area_threshold=self.cfg_cg.mask_area_threshold,
            max_bbox_area_ratio=self.cfg_cg.max_bbox_area_ratio,
            mask_conf_threshold=self.cfg_cg.mask_conf_threshold,
        )
        gobs = filtered_gobs
        if len(gobs["mask"]) == 0:
            logging.debug("No detections left after filter_gobs")
            return (image_rgb, [])
        gobs["mask"] = mask_subtract_contained(gobs["xyxy"], gobs["mask"])
        obj_pcds_and_bboxes = measure_time(detections_to_obj_pcd_and_bbox)(
            depth_array=depth,
            masks=gobs["mask"],
            cam_K=intrinsics[:3, :3],
            image_rgb=image_rgb,
            trans_pose=cam_pose,
            min_points_threshold=self.cfg_cg.min_points_threshold,
            spatial_sim_type=self.cfg_cg.spatial_sim_type,
            obj_pcd_max_points=self.cfg_cg.obj_pcd_max_points,
            device=self.device,
        )
        for obj in obj_pcds_and_bboxes:
            if obj:
                obj["pcd"] = init_process_pcd(
                    pcd=obj["pcd"],
                    downsample_voxel_size=self.cfg_cg["downsample_voxel_size"],
                    dbscan_remove_noise=self.cfg_cg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg_cg["dbscan_eps"],
                    dbscan_min_points=self.cfg_cg["dbscan_min_points"],
                )
                obj["bbox"] = get_bounding_box(
                    spatial_sim_type=self.cfg_cg["spatial_sim_type"], pcd=obj["pcd"]
                )
        if all([obj is None for obj in obj_pcds_and_bboxes]):
            logging.debug("All objects are None in obj_pcds_and_bboxes")
            return (image_rgb, [])
        gobs["bbox"] = [
            obj["bbox"] if obj is not None else None for obj in obj_pcds_and_bboxes
        ]
        gobs["pcd"] = [
            obj["pcd"] if obj is not None else None for obj in obj_pcds_and_bboxes
        ]
        gobs = self.filter_gobs_with_distance(pts, gobs)
        detection_list = self.make_detection_list_from_pcd_and_gobs(
            gobs, img_path, obj_classes
        )
        if len(detection_list) == 0:
            logging.debug(
                "No detections left after make_detection_list_from_pcd_and_gobs"
            )
            return (image_rgb, [])
        frame = SnapShot(
            image=img_path,
            color=(random.random(), random.random(), random.random()),
            pos=pts,
            angle=angle,
            depth=depth,
            pose=cam_pose,
        )
        frame.full_obj_list = {
            obj_id: detection_list[obj_id]["conf"] for obj_id in detection_list.keys()
        }
        det_visual_prompt = sv.Detections(xyxy=gobs["xyxy"], class_id=gobs["class_id"])
        if len(self.objects) == 0:
            logging.debug(
                f"No objects in the map yet, adding all detections of length {len(detection_list)}"
            )
            self.objects.update(detection_list)
            det_visual_prompt.data["obj_id"] = list(detection_list.keys())
            frame.visual_prompt = det_visual_prompt
            self.frames[img_path] = frame
            visualize_captions = [
                f"{detected_obj_id} {detection_list[detected_obj_id]['class_name']} {detection_list[detected_obj_id]['conf']:.3f} N"
                for detected_obj_id in detection_list.keys()
            ]
            det_visualize = sv.Detections(
                xyxy=gobs["xyxy"],
                confidence=gobs["confidence"],
                class_id=gobs["class_id"],
            )
            det_visualize.data["class_name"] = visualize_captions
            annotated_image = image_rgb.copy()
            BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=1)
            LABEL_ANNOTATOR = sv.LabelAnnotator(
                text_thickness=1, text_scale=0.25, text_color=sv.Color.BLACK
            )
            annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(
                annotated_image, det_visualize
            )
            annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, det_visualize)
            added_obj_ids = list(detection_list.keys())
        else:
            spatial_sim = compute_spatial_similarities(
                spatial_sim_type=self.cfg_cg["spatial_sim_type"],
                detection_list=detection_list,
                objects=self.objects,
                downsample_voxel_size=self.cfg_cg["downsample_voxel_size"],
            )
            visual_sim = compute_visual_similarities(detection_list, self.objects)
            agg_sim = aggregate_similarities(
                match_method=self.cfg_cg["match_method"],
                phys_bias=self.cfg_cg["phys_bias"],
                spatial_sim=spatial_sim,
                visual_sim=visual_sim,
            )
            match_indices = match_detections_to_objects(
                agg_sim=agg_sim,
                detection_threshold=self.cfg_cg["sim_threshold"],
                existing_obj_ids=list(self.objects.keys()),
                detected_obj_ids=list(detection_list.keys()),
            )
            visualize_captions, added_obj_ids, all_obj_ids = self.merge_obj_matches(
                detection_list=detection_list,
                match_indices=match_indices,
                obj_classes=obj_classes,
                snapshot=frame,
            )
            det_visual_prompt.data["obj_id"] = all_obj_ids
            frame.visual_prompt = det_visual_prompt
            self.frames[img_path] = frame
            det_visualize = sv.Detections(
                xyxy=gobs["xyxy"],
                confidence=gobs["confidence"],
                class_id=gobs["class_id"],
            )
            det_visualize.data["class_name"] = visualize_captions
            annotated_image = image_rgb.copy()
            BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=1)
            LABEL_ANNOTATOR = sv.LabelAnnotator(
                text_thickness=1, text_scale=0.25, text_color=sv.Color.BLACK
            )
            annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(
                annotated_image, det_visualize
            )
            annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, det_visualize)
        return (annotated_image, added_obj_ids)

    def filter_gobs_with_distance(self, pts, gobs):
        idx_to_keep = []
        for idx in range(len(gobs["bbox"])):
            if gobs["bbox"][idx] is None:
                continue
            if (
                np.linalg.norm(gobs["bbox"][idx].center[[0, 1]] - pts[[0, 1]])
                > self.cfg.scene_graph.obj_include_dist
            ):
                logging.info(
                    f"Object {gobs['detection_class_labels'][idx]} rejected by distance limit {self.cfg.scene_graph.obj_include_dist}m"
                )
                continue
            idx_to_keep.append(idx)
        for attribute in gobs.keys():
            if isinstance(gobs[attribute], str) or attribute == "classes":
                continue
            if attribute in ["labels", "edges", "text_feats", "captions"]:
                continue
            elif isinstance(gobs[attribute], list):
                gobs[attribute] = [gobs[attribute][i] for i in idx_to_keep]
            elif isinstance(gobs[attribute], np.ndarray):
                gobs[attribute] = gobs[attribute][idx_to_keep]
            else:
                raise NotImplementedError(f"Unhandled type {type(gobs[attribute])}")
        return gobs

    def merge_obj_matches(
        self,
        detection_list: DetectionDict,
        match_indices: List[Tuple[int, Optional[int]]],
        obj_classes: ObjectClasses,
        snapshot: SnapShot,
    ):
        visualize_captions = []
        all_obj_ids = []
        added_obj_ids = []
        for idx, (detected_obj_id, existing_obj_match_id) in enumerate(match_indices):
            if existing_obj_match_id is None:
                self.objects[detected_obj_id] = detection_list[detected_obj_id]
                visualize_captions.append(
                    f"{detected_obj_id} {self.objects[detected_obj_id]['class_name']} {self.objects[detected_obj_id]['conf']:.3f} N"
                )
                all_obj_ids.append(detected_obj_id)
                added_obj_ids.append(detected_obj_id)
            else:
                detected_obj = detection_list[detected_obj_id]
                matched_obj = self.objects[existing_obj_match_id]
                merged_obj = merge_obj2_into_obj1(
                    obj1=matched_obj,
                    obj2=detected_obj,
                    downsample_voxel_size=self.cfg_cg["downsample_voxel_size"],
                    dbscan_remove_noise=self.cfg_cg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg_cg["dbscan_eps"],
                    dbscan_min_points=self.cfg_cg["dbscan_min_points"],
                    spatial_sim_type=self.cfg_cg["spatial_sim_type"],
                    device=self.device,
                    run_dbscan=False,
                )
                class_id_counter = Counter(merged_obj["class_id"])
                most_common_class_id = class_id_counter.most_common(1)[0][0]
                most_common_class_name = obj_classes.get_classes_arr()[
                    most_common_class_id
                ]
                merged_obj["class_name"] = most_common_class_name
                snapshot.full_obj_list[existing_obj_match_id] = detected_obj["conf"]
                snapshot.full_obj_list.pop(detected_obj_id)
                self.objects[existing_obj_match_id] = merged_obj
                visualize_captions.append(
                    f"{existing_obj_match_id} {self.objects[existing_obj_match_id]['class_name']} {detected_obj['conf']:.3f} {merged_obj['num_detections']}"
                )
                all_obj_ids.append(existing_obj_match_id)
        return (visualize_captions, added_obj_ids, all_obj_ids)

    def make_detection_list_from_pcd_and_gobs(
        self, gobs, image_path, obj_classes
    ) -> DetectionDict:
        detection_list = DetectionDict()
        for mask_idx in range(len(gobs["mask"])):
            if gobs["pcd"][mask_idx] is None:
                continue
            curr_class_name = gobs["classes"][gobs["class_id"][mask_idx]]
            curr_class_idx = obj_classes.get_classes_arr().index(curr_class_name)
            detected_object = {
                "id": self.object_id_counter,
                "class_name": curr_class_name,
                "class_id": [curr_class_idx],
                "num_detections": 1,
                "conf": gobs["confidence"][mask_idx],
                "pcd": gobs["pcd"][mask_idx],
                "bbox": gobs["bbox"][mask_idx],
                "clip_ft": to_tensor(gobs["image_feats"][mask_idx]),
                "image": None,
            }
            detection_list[self.object_id_counter] = detected_object
            self.object_id_counter += 1
        return detection_list

    def cleanup_empty_frames_snapshots(self):
        filtered_frames = {}
        for file_name, frame in self.frames.items():
            if len(frame.full_obj_list) > 0:
                filtered_frames[file_name] = frame
        self.frames = filtered_frames
        filtered_snapshots = {}
        for file_name, snapshot in self.snapshots.items():
            if len(snapshot.cluster) > 0:
                filtered_snapshots[file_name] = snapshot
        self.snapshots = filtered_snapshots

    def update_snapshots(self, obj_ids, min_detection=2):
        self.cleanup_empty_frames_snapshots()
        prev_snapshots = copy.deepcopy(self.snapshots)
        obj_ids_temp = obj_ids.copy()
        for filename, snapshot in self.snapshots.items():
            cluster = snapshot.cluster
            if any([obj_id in obj_ids_temp for obj_id in cluster]):
                obj_ids = obj_ids.union(set(cluster))
                prev_snapshots.pop(filename)
        obj_ids = list(set(obj_ids))
        obj_exclude = [
            obj_id
            for obj_id in self.objects.keys()
            if self.objects[obj_id]["num_detections"] < min_detection
        ]
        obj_ids = [obj_id for obj_id in obj_ids if obj_id not in obj_exclude]
        obj_centers = np.zeros((len(obj_ids), 2))
        for i, obj_id in enumerate(obj_ids):
            obj_centers[i] = self.objects[obj_id]["bbox"].center[[0, 2]]
        if len(obj_centers) == 0:
            return
        new_snapshots = self.clustering.fit(obj_centers, obj_ids, self.frames)
        prev_snapshot_objs = [
            obj_id
            for snapshot in prev_snapshots.values()
            for obj_id in snapshot.cluster
        ]
        assert set(
            [
                obj_id
                for snapshot in new_snapshots.values()
                for obj_id in snapshot.cluster
            ]
        ) == set(
            obj_ids
        ), f"{set([obj_id for snapshot in new_snapshots.values() for obj_id in snapshot.cluster])} != {set(obj_ids)}"
        assert (
            set(obj_ids) & set(prev_snapshot_objs) == set()
        ), f"{set(obj_ids)} & {set(prev_snapshot_objs)} != empty"
        assert set(obj_ids) | set(prev_snapshot_objs) | set(obj_exclude) == set(
            self.objects.keys()
        ), f"{set(obj_ids)} | {set(prev_snapshot_objs)} | {set(obj_exclude)} != {set(self.objects.keys())}"
        for key, snapshot in new_snapshots.items():
            if key in prev_snapshots.keys():
                prev_snapshots[key].cluster += snapshot.cluster
            else:
                prev_snapshots[key] = snapshot
        self.snapshots = prev_snapshots
        for file_name, snapshot in self.snapshots.items():
            for obj_id in snapshot.cluster:
                self.objects[obj_id]["image"] = file_name
        for file_name, snapshot in self.snapshots.items():
            self.frames[file_name] = snapshot
        for obj_id, obj in self.objects.items():
            if obj["num_detections"] < min_detection:
                assert (
                    obj["image"] is None
                ), f"{obj_id} has only one detection but has image"
            else:
                assert obj["image"] is not None, f"{obj_id} has no image"

    def periodic_cleanup_objects(self, frame_idx, pts, goal_obj_ids_mapping=None):
        if processing_needed(
            self.cfg_cg["denoise_interval"],
            self.cfg_cg["run_denoise_final_frame"],
            frame_idx,
            is_final_frame=False,
        ):
            self.objects = measure_time(denoise_objects)(
                downsample_voxel_size=self.cfg_cg["downsample_voxel_size"],
                dbscan_remove_noise=self.cfg_cg["dbscan_remove_noise"],
                dbscan_eps=self.cfg_cg["dbscan_eps"],
                dbscan_min_points=self.cfg_cg["dbscan_min_points"],
                spatial_sim_type=self.cfg_cg["spatial_sim_type"],
                device=self.device,
                objects=self.objects,
            )
        if processing_needed(
            self.cfg_cg["filter_interval"],
            self.cfg_cg["run_filter_final_frame"],
            frame_idx,
            is_final_frame=False,
        ):
            self.objects = filter_objects(
                obj_min_points=self.cfg_cg["obj_min_points"],
                obj_min_detections=self.cfg_cg["obj_min_detections"],
                min_distance=self.cfg.scene_graph.obj_include_dist,
                objects=self.objects,
                pts=pts,
            )
        if processing_needed(
            self.cfg_cg["merge_interval"],
            self.cfg_cg["run_merge_final_frame"],
            frame_idx,
            is_final_frame=False,
        ):
            self.objects = measure_time(merge_objects)(
                merge_overlap_thresh=self.cfg_cg["merge_overlap_thresh"],
                merge_visual_sim_thresh=self.cfg_cg["merge_visual_sim_thresh"],
                merge_text_sim_thresh=self.cfg_cg["merge_text_sim_thresh"],
                objects=self.objects,
                downsample_voxel_size=self.cfg_cg["downsample_voxel_size"],
                dbscan_remove_noise=self.cfg_cg["dbscan_remove_noise"],
                dbscan_eps=self.cfg_cg["dbscan_eps"],
                dbscan_min_points=self.cfg_cg["dbscan_min_points"],
                spatial_sim_type=self.cfg_cg["spatial_sim_type"],
                device=self.device,
                goal_obj_ids_mapping=goal_obj_ids_mapping,
            )
        frame_to_pop = []
        for filename, ss in self.frames.items():
            ss.cluster = [
                obj_id for obj_id in ss.cluster if obj_id in self.objects.keys()
            ]
            ss.full_obj_list = {
                obj_id: conf
                for obj_id, conf in ss.full_obj_list.items()
                if obj_id in self.objects.keys()
            }
            if len(ss.full_obj_list) == 0:
                frame_to_pop.append(filename)
        for filename in frame_to_pop:
            self.frames.pop(filename)
        if goal_obj_ids_mapping is not None:
            for goal_obj_id, mapped_obj_ids in goal_obj_ids_mapping.items():
                goal_obj_ids_mapping[goal_obj_id] = [
                    obj_id for obj_id in mapped_obj_ids if obj_id in self.objects.keys()
                ]

    def sanity_check(self, cfg):
        obj_exclude_count = sum(
            [
                1 if obj["num_detections"] < cfg.min_detection else 0
                for obj in self.objects.values()
            ]
        )
        total_objs_count = sum(
            [len(snapshot.cluster) for snapshot in self.snapshots.values()]
        )
        assert (
            len(self.objects) == total_objs_count + obj_exclude_count
        ), f"{len(self.objects)} != {total_objs_count} + {obj_exclude_count}"
        total_objs_count = sum(
            [len(set(snapshot.cluster)) for snapshot in self.snapshots.values()]
        )
        assert (
            len(self.objects) == total_objs_count + obj_exclude_count
        ), f"{len(self.objects)} != {total_objs_count} + {obj_exclude_count}"
        for obj_id in self.objects.keys():
            exist_count = 0
            for ss in self.snapshots.values():
                if obj_id in ss.cluster:
                    exist_count += 1
            if self.objects[obj_id]["num_detections"] < cfg.min_detection:
                assert (
                    exist_count == 0
                ), f"{exist_count} != 0 for obj_id {obj_id}, {self.objects[obj_id]['class_name']}"
            else:
                assert (
                    exist_count == 1
                ), f"{exist_count} != 1 for obj_id {obj_id}, {self.objects[obj_id]['class_name']}"
        for ss in self.snapshots.values():
            assert len(ss.cluster) == len(
                set(ss.cluster)
            ), f"{ss.cluster} has duplicates"
            assert len(ss.full_obj_list.keys()) == len(
                set(ss.full_obj_list.keys())
            ), f"{ss.full_obj_list.keys()} has duplicates"
            for obj_id in ss.cluster:
                assert (
                    obj_id in ss.full_obj_list
                ), f"{obj_id} not in {ss.full_obj_list.keys()}"
            for obj_id in ss.full_obj_list.keys():
                assert obj_id in self.objects, f"{obj_id} not in scene objects"
        for file_name, ss in self.snapshots.items():
            assert (
                ss.cluster == self.frames[file_name].cluster
            ), f"{ss}\n!=\n{self.frames[file_name]}"
            assert (
                ss.full_obj_list == self.frames[file_name].full_obj_list
            ), f"{ss}\n==\n{self.frames[file_name]}"

    def print_scene_graph(self):
        snapshot_dict = {}
        for obj_id, obj in self.objects.items():
            if obj["image"] not in snapshot_dict:
                snapshot_dict[obj["image"]] = []
            snapshot_dict[obj["image"]].append(
                f"{obj_id}: {obj['class_name']} {obj['num_detections']}"
            )
        for snapshot_id, obj_list in snapshot_dict.items():
            logging.info(f"{snapshot_id}:")
            for obj_str in obj_list:
                logging.info(f"\t{obj_str}")


class AirsimPathfinder:

    def __init__(self, airsim_bridge, init_pts_memory):
        self.airsim_bridge = airsim_bridge
        self.seed_value = 77
        center_x, center_y, center_z = (
            init_pts_memory[0],
            init_pts_memory[1],
            init_pts_memory[2],
        )
        self.ground_level = 0.0
        self.min_flight_height = 1.0
        self.max_flight_height = 15.0
        self.bounds = {
            "x_min": center_x - 50,
            "x_max": center_x + 50,
            "y_min": self.min_flight_height,
            "y_max": self.max_flight_height,
            "z_min": center_z - 50,
            "z_max": center_z + 50,
        }
        self.navigable_points_cache = []
        self.cache_size = 1000
        self._populate_navigable_cache()
        logging.info(f"AirSim pathfinder initialized with bounds: {self.bounds}")

    def seed(self, seed_val):
        self.seed_value = seed_val
        np.random.seed(seed_val)
        random.seed(seed_val)

    def _populate_navigable_cache(self):
        self.navigable_points_cache = []
        for _ in range(self.cache_size):
            x = np.random.uniform(self.bounds["x_min"], self.bounds["x_max"])
            y = np.random.uniform(self.bounds["y_min"], self.bounds["y_max"])
            z = np.random.uniform(self.bounds["z_min"], self.bounds["z_max"])
            point = np.array([x, y, z])
            if self.is_navigable(point):
                self.navigable_points_cache.append(point)
        logging.info(f"Generated {len(self.navigable_points_cache)} navigable points")

    def get_random_navigable_point(self):
        if len(self.navigable_points_cache) < 10:
            self._populate_navigable_cache()
        if len(self.navigable_points_cache) == 0:
            logging.warning(
                "No navigable points in cache, returning safe fallback point"
            )
            return np.array([0.0, 0.0, -2.0])
        return self.navigable_points_cache[
            np.random.randint(len(self.navigable_points_cache))
        ]

    def get_random_navigable_point_near(self, circle_center, radius):
        max_attempts = 50
        for _ in range(max_attempts):
            angle = np.random.uniform(0, 2 * np.pi)
            r = np.random.uniform(0, radius)
            offset = np.array(
                [r * np.cos(angle), r * np.sin(angle), np.random.uniform(-1.0, 1.0)]
            )
            point = circle_center + offset
            if self.is_navigable(point):
                return point
        if self.is_navigable(circle_center):
            return circle_center
        else:
            return self.snap_point(circle_center)

    def is_navigable(self, point):
        if not (
            self.bounds["x_min"] <= point[0] <= self.bounds["x_max"]
            and self.bounds["y_min"] <= point[1] <= self.bounds["y_max"]
            and (self.bounds["z_min"] <= point[2] <= self.bounds["z_max"])
        ):
            return False
        return True

    def snap_point(self, point):
        snapped = np.array(
            [
                np.clip(point[0], self.bounds["x_min"], self.bounds["x_max"]),
                np.clip(point[1], self.bounds["y_min"], self.bounds["y_max"]),
                np.clip(point[2], self.bounds["z_min"], self.bounds["z_max"]),
            ]
        )
        return snapped

    def find_path(self, path_request):
        start = path_request.requested_start
        end = path_request.requested_end
        if not self.is_navigable(start):
            start = self.snap_point(start)
            logging.warning(f"Start point snapped to: {start}")
        if not self.is_navigable(end):
            end = self.snap_point(end)
            logging.warning(f"End point snapped to: {end}")
        distance = np.linalg.norm(end - start)
        num_waypoints = max(2, int(distance / 2.0))
        path_points = []
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            waypoint = start + t * (end - start)
            if waypoint[1] <= self.ground_level:
                waypoint[1] = self.min_flight_height
                logging.warning(f"Waypoint adjusted to stay above ground: {waypoint}")
            path_points.append(waypoint)
        path_request.points = path_points
        path_request.geodesic_distance = distance
        print(f"path point {path_points}")
        return True


class AirsimObjectClasses:

    def __init__(self, bg_classes=None, skip_bg=True, class_set="scannet200"):
        self.bg_classes = bg_classes or []
        self.skip_bg = skip_bg
        self.classes = [
            "car",
            "chair",
            "table",
            "bench",
            "pool",
            "building",
            "tree",
            "person",
            "bicycle",
            "motorcycle",
            "bus",
            "truck",
            "traffic light",
            "fire hydrant",
            "stop sign",
            "parking meter",
            "trash can",
            "lamp post",
            "turtle",
            "rabbit",
            "bridge",
            "fruit store",
            "fountain",
            "boat",
            "food store",
            "food",
            "umbrella",
        ]

    def get_classes_arr(self):
        return self.classes

    def get_bg_classes_arr(self):
        return self.bg_classes

    def get_classes_set(self):
        return set(self.classes)
