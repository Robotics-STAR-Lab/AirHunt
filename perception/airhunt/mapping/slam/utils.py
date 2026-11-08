from collections import Counter
import logging
from airhunt.mapping.utils.logging_metrics import MappingTracker
import cv2
import numpy as np
import open3d as o3d
import torch
import typing as tp
import torch.nn.functional as F
import faiss
from airhunt.mapping.slam.slam_classes import to_tensor, MapObjectDict
from airhunt.mapping.utils.ious import compute_3d_iou_accurate_batch

tracker = MappingTracker()


def pcd_denoise_dbscan(
    pcd: o3d.geometry.PointCloud, eps=0.02, min_points=10
) -> o3d.geometry.PointCloud:
    pcd_clusters = pcd.cluster_dbscan(eps=eps, min_points=min_points)
    obj_points = np.asarray(pcd.points)
    obj_colors = np.asarray(pcd.colors)
    pcd_clusters = np.array(pcd_clusters)
    counter = Counter(pcd_clusters)
    if counter and -1 in counter:
        del counter[-1]
    if counter:
        most_common_label, _ = counter.most_common(1)[0]
        largest_mask = pcd_clusters == most_common_label
        largest_cluster_points = obj_points[largest_mask]
        largest_cluster_colors = obj_colors[largest_mask]
        if len(largest_cluster_points) < 5:
            return pcd
        largest_cluster_pcd = o3d.geometry.PointCloud()
        largest_cluster_pcd.points = o3d.utility.Vector3dVector(largest_cluster_points)
        largest_cluster_pcd.colors = o3d.utility.Vector3dVector(largest_cluster_colors)
        pcd = largest_cluster_pcd
    return pcd


def init_pcd_denoise_dbscan(
    pcd: o3d.geometry.PointCloud, eps=0.02, min_points=10
) -> o3d.geometry.PointCloud:
    pcd_clusters = pcd.cluster_dbscan(eps=eps, min_points=min_points)
    obj_points = np.asarray(pcd.points)
    obj_colors = np.asarray(pcd.colors)
    pcd_clusters = np.array(pcd_clusters)
    counter = Counter(pcd_clusters)
    if counter and -1 in counter:
        del counter[-1]
    if counter:
        most_common_label, _ = counter.most_common(1)[0]
        largest_mask = pcd_clusters == most_common_label
        largest_cluster_points = obj_points[largest_mask]
        largest_cluster_colors = obj_colors[largest_mask]
        if len(largest_cluster_points) < 5:
            return pcd
        largest_cluster_pcd = o3d.geometry.PointCloud()
        largest_cluster_pcd.points = o3d.utility.Vector3dVector(largest_cluster_points)
        largest_cluster_pcd.colors = o3d.utility.Vector3dVector(largest_cluster_colors)
        pcd = largest_cluster_pcd
    return pcd


def init_process_pcd(
    pcd,
    downsample_voxel_size,
    dbscan_remove_noise,
    dbscan_eps,
    dbscan_min_points,
    run_dbscan=True,
):
    pcd = pcd.voxel_down_sample(voxel_size=downsample_voxel_size)
    if dbscan_remove_noise and run_dbscan:
        pcd = init_pcd_denoise_dbscan(pcd, eps=dbscan_eps, min_points=dbscan_min_points)
    return pcd


def process_pcd(
    pcd,
    downsample_voxel_size,
    dbscan_remove_noise,
    dbscan_eps,
    dbscan_min_points,
    run_dbscan=True,
):
    pcd = pcd.voxel_down_sample(voxel_size=downsample_voxel_size)
    if dbscan_remove_noise and run_dbscan:
        pass
        pcd = pcd_denoise_dbscan(pcd, eps=dbscan_eps, min_points=dbscan_min_points)
    return pcd


def get_bounding_box(spatial_sim_type, pcd):
    if ("accurate" in spatial_sim_type or "overlap" in spatial_sim_type) and len(
        pcd.points
    ) >= 4:
        try:
            return pcd.get_oriented_bounding_box(robust=True)
        except RuntimeError as e:
            print(f"Met {e}, use axis aligned bounding box instead")
            return pcd.get_axis_aligned_bounding_box()
    else:
        return pcd.get_axis_aligned_bounding_box()


def merge_obj2_into_obj1(
    obj1,
    obj2,
    downsample_voxel_size,
    dbscan_remove_noise,
    dbscan_eps,
    dbscan_min_points,
    spatial_sim_type,
    device,
    run_dbscan=True,
):
    global tracker
    tracker.track_merge(obj1, obj2)
    extend_attributes = ["class_id"]
    add_attributes = ["num_detections"]
    skip_attributes = ["id", "class_name"]
    custom_handled = ["pcd", "bbox", "clip_ft", "conf", "image", "image_crop"]
    all_handled_keys = set(
        extend_attributes + add_attributes + skip_attributes + custom_handled
    )
    unhandled_keys = set(obj2.keys()) - all_handled_keys
    if unhandled_keys:
        raise ValueError(
            f"Unhandled keys detected in obj2: {unhandled_keys}. Please update the merge function to handle these attributes."
        )
    n_obj1_det = obj1["num_detections"]
    n_obj2_det = obj2["num_detections"]
    for attr in extend_attributes:
        if attr in obj1 and attr in obj2:
            obj1[attr].extend(obj2[attr])
    for attr in add_attributes:
        if attr in obj1 and attr in obj2:
            obj1[attr] += obj2[attr]
    obj1["pcd"] += obj2["pcd"]
    obj1["pcd"] = process_pcd(
        obj1["pcd"],
        downsample_voxel_size,
        dbscan_remove_noise,
        dbscan_eps,
        dbscan_min_points,
        run_dbscan,
    )
    obj1["bbox"] = get_bounding_box(spatial_sim_type, obj1["pcd"])
    obj1["bbox"].color = [0, 1, 0]
    obj1["clip_ft"] = (obj1["clip_ft"] * n_obj1_det + obj2["clip_ft"] * n_obj2_det) / (
        n_obj1_det + n_obj2_det
    )
    obj1["clip_ft"] = F.normalize(obj1["clip_ft"], dim=0)
    if "image_crop" in obj1 and "image_crop" in obj2:
        if obj1["conf"] < obj2["conf"]:
            obj1["image_crop"] = obj2["image_crop"]
    obj1["conf"] = max(obj1["conf"], obj2["conf"])
    return obj1


def compute_overlap_matrix_general(
    objects_a: MapObjectDict, objects_b=None, downsample_voxel_size=None
) -> np.ndarray:
    if downsample_voxel_size is None:
        raise ValueError("downsample_voxel_size is not provided")
    same_objects = objects_b is None
    objects_b = objects_a if same_objects else objects_b
    len_a = len(objects_a)
    len_b = len(objects_b)
    overlap_matrix = np.zeros((len_a, len_b))
    points_a = [
        np.asarray(obj["pcd"].points, dtype=np.float32) for obj in objects_a.values()
    ]
    indices_a = [faiss.IndexFlatL2(points_a_arr.shape[1]) for points_a_arr in points_a]
    for idx_a, points_a_arr in zip(indices_a, points_a):
        idx_a.add(points_a_arr)
    points_b = [
        np.asarray(obj["pcd"].points, dtype=np.float32) for obj in objects_b.values()
    ]
    bbox_a = objects_a.get_stacked_values_torch("bbox")
    bbox_b = objects_b.get_stacked_values_torch("bbox")
    ious = compute_3d_iou_accurate_batch(bbox_a, bbox_b)
    for idx_a in range(len_a):
        for idx_b in range(len_b):
            if same_objects and idx_a == idx_b:
                continue
            if ious[idx_a, idx_b] < 1e-06:
                continue
            D, I = indices_a[idx_a].search(points_b[idx_b], 1)
            overlap = (D < downsample_voxel_size**2).sum()
            overlap_matrix[idx_a, idx_b] = overlap / len(points_b[idx_b])
    return overlap_matrix


def merge_overlap_objects(
    merge_overlap_thresh: float,
    merge_visual_sim_thresh: float,
    merge_text_sim_thresh: float,
    objects: MapObjectDict,
    overlap_matrix: np.ndarray,
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    spatial_sim_type: str,
    device: str,
    goal_obj_ids_mapping: tp.Dict[int, tp.List[int]] = None,
):
    x, y = overlap_matrix.nonzero()
    overlap_ratio = overlap_matrix[x, y]
    sort = np.argsort(overlap_ratio)[::-1]
    x = x[sort]
    y = y[sort]
    overlap_ratio = overlap_ratio[sort]
    obj_id_list = list(objects.keys())
    for i, j, ratio in zip(x, y, overlap_ratio):
        if ratio > merge_overlap_thresh:
            obj_i = obj_id_list[i]
            obj_j = obj_id_list[j]
            if obj_i not in objects or obj_j not in objects:
                continue
            visual_sim = F.cosine_similarity(
                to_tensor(objects[obj_i]["clip_ft"]),
                to_tensor(objects[obj_j]["clip_ft"]),
                dim=0,
            )
            text_sim = visual_sim
            if (
                visual_sim > merge_visual_sim_thresh
                and text_sim > merge_text_sim_thresh
            ):
                if obj_i in objects and obj_j in objects:
                    if objects[obj_j]["conf"] < objects[obj_i]["conf"]:
                        obj_i, obj_j = (obj_j, obj_i)
                    objects[obj_j] = merge_obj2_into_obj1(
                        objects[obj_j],
                        objects[obj_i],
                        downsample_voxel_size,
                        dbscan_remove_noise,
                        dbscan_eps,
                        dbscan_min_points,
                        spatial_sim_type,
                        device,
                        run_dbscan=True,
                    )
                    objects.pop(obj_i)
                    if goal_obj_ids_mapping is not None:
                        for goal_obj_id, mapped_obj_ids in goal_obj_ids_mapping.items():
                            while obj_i in mapped_obj_ids:
                                mapped_obj_ids.remove(obj_i)
                                mapped_obj_ids.append(obj_j)
                            goal_obj_ids_mapping[goal_obj_id] = mapped_obj_ids
        else:
            break
    return objects


def denoise_objects(
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    spatial_sim_type: str,
    device: str,
    objects: MapObjectDict,
):
    logging.debug(f"Starting denoising with {len(objects)} objects")
    for obj_id in objects.keys():
        og_object_pcd = objects[obj_id]["pcd"]
        if len(og_object_pcd.points) <= 1:
            objects[obj_id]["pcd"] = og_object_pcd
        else:
            objects[obj_id]["pcd"] = process_pcd(
                objects[obj_id]["pcd"],
                downsample_voxel_size,
                dbscan_remove_noise,
                dbscan_eps,
                dbscan_min_points,
                run_dbscan=True,
            )
            if len(objects[obj_id]["pcd"].points) < 4:
                objects[obj_id]["pcd"] = og_object_pcd
        objects[obj_id]["bbox"] = get_bounding_box(
            spatial_sim_type, objects[obj_id]["pcd"]
        )
        objects[obj_id]["bbox"].color = [0, 1, 0]
        logging.debug(f"Finished denoising object {obj_id} out of {len(objects)}")
        logging.debug(
            f"before denoising: {len(og_object_pcd.points)}, after denoising: {len(objects[obj_id]['pcd'].points)}"
        )
    logging.debug(f"Finished denoising with {len(objects)} objects")
    return objects


def filter_objects(
    obj_min_points: int,
    obj_min_detections: int,
    min_distance: float,
    objects: MapObjectDict,
    pts: np.ndarray,
):
    print("Before filtering:", len(objects))
    objects_to_keep = MapObjectDict()
    for obj_id, obj in objects.items():
        if np.linalg.norm(obj["bbox"].center[[0, 2]] - pts[[0, 2]]) > min_distance:
            if (
                len(obj["pcd"].points) >= obj_min_points
                and obj["num_detections"] >= obj_min_detections
            ):
                objects_to_keep[obj_id] = obj
            else:
                logging.debug(
                    f"Filtered object {obj_id} {obj['class_name']} in {obj['image']}"
                )
        else:
            objects_to_keep[obj_id] = obj
    print("After filtering:", len(objects_to_keep))
    return objects_to_keep


def merge_objects(
    merge_overlap_thresh: float,
    merge_visual_sim_thresh: float,
    merge_text_sim_thresh: float,
    objects: MapObjectDict,
    downsample_voxel_size: float,
    dbscan_remove_noise: bool,
    dbscan_eps: float,
    dbscan_min_points: int,
    spatial_sim_type: str,
    device: str,
    goal_obj_ids_mapping: dict = None,
):
    if len(objects) == 0:
        return objects
    if merge_overlap_thresh <= 0:
        return objects
    overlap_matrix = compute_overlap_matrix_general(
        objects_a=objects, objects_b=None, downsample_voxel_size=downsample_voxel_size
    )
    print("Before merging:", len(objects))
    objects = merge_overlap_objects(
        merge_overlap_thresh=merge_overlap_thresh,
        merge_visual_sim_thresh=merge_visual_sim_thresh,
        merge_text_sim_thresh=merge_text_sim_thresh,
        objects=objects,
        overlap_matrix=overlap_matrix,
        downsample_voxel_size=downsample_voxel_size,
        dbscan_remove_noise=dbscan_remove_noise,
        dbscan_eps=dbscan_eps,
        dbscan_min_points=dbscan_min_points,
        spatial_sim_type=spatial_sim_type,
        device=device,
        goal_obj_ids_mapping=goal_obj_ids_mapping,
    )
    print("After merging:", len(objects))
    return objects


def filter_gobs(
    gobs: dict,
    image: np.ndarray,
    skip_bg: bool = None,
    BG_CLASSES: list = None,
    mask_area_threshold: float = 10,
    max_bbox_area_ratio: float = None,
    mask_conf_threshold: float = None,
):
    if len(gobs["xyxy"]) == 0:
        return gobs
    idx_to_keep = []
    for mask_idx in range(len(gobs["xyxy"])):
        local_class_id = gobs["class_id"][mask_idx]
        class_name = gobs["classes"][local_class_id]
        mask_area = gobs["mask"][mask_idx].sum()
        if mask_area < max(mask_area_threshold, 10):
            logging.info(
                f"Skipped due to small mask area ({mask_area} pixels) - Class: {class_name}"
            )
            continue
        if skip_bg and class_name in BG_CLASSES:
            logging.info(f"Skipped background class: {class_name}")
            continue
        if class_name not in BG_CLASSES:
            x1, y1, x2, y2 = gobs["xyxy"][mask_idx]
            bbox_area = (x2 - x1) * (y2 - y1)
            image_area = image.shape[0] * image.shape[1]
            if (
                max_bbox_area_ratio is not None
                and bbox_area > max_bbox_area_ratio * image_area
            ):
                logging.info(
                    f"Skipped due to large bounding box area ratio - Class: {class_name}, Area Ratio: {bbox_area / image_area:.4f}"
                )
                continue
        if mask_conf_threshold is not None and gobs["confidence"] is not None:
            if gobs["confidence"][mask_idx] < mask_conf_threshold:
                logging.info(
                    f"Skipped due to low confidence ({gobs['confidence'][mask_idx]}) - Class: {class_name}"
                )
                continue
        idx_to_keep.append(mask_idx)
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


def resize_gobs(gobs, image):
    if gobs["mask"].shape[1:] == image.shape[:2]:
        return gobs
    new_masks = []
    for mask_idx in range(len(gobs["xyxy"])):
        mask = gobs["mask"][mask_idx]
        x1, y1, x2, y2 = gobs["xyxy"][mask_idx]
        x1 = round(x1 * image.shape[1] / mask.shape[1])
        y1 = round(y1 * image.shape[0] / mask.shape[0])
        x2 = round(x2 * image.shape[1] / mask.shape[1])
        y2 = round(y2 * image.shape[0] / mask.shape[0])
        gobs["xyxy"][mask_idx] = [x1, y1, x2, y2]
        mask = cv2.resize(
            mask.astype(np.uint8),
            image.shape[:2][::-1],
            interpolation=cv2.INTER_NEAREST,
        )
        mask = mask.astype(bool)
        new_masks.append(mask)
    if len(new_masks) > 0:
        gobs["mask"] = np.asarray(new_masks)
    return gobs


def dynamic_downsample(points, colors=None, target=5000):
    if target == -1:
        return (points, colors)
    num_points = points.size(0)
    if num_points <= target:
        return (points, colors)
    downsample_factor = max(1, num_points // target)
    downsampled_points = points[::downsample_factor]
    downsampled_colors = colors[::downsample_factor] if colors is not None else None
    return (downsampled_points, downsampled_colors)


def batch_mask_depth_to_points_colors(
    depth_tensor: torch.Tensor,
    masks_tensor: torch.Tensor,
    cam_K: torch.Tensor,
    image_rgb_tensor: torch.Tensor = None,
    device: str = "cuda",
) -> tuple:
    N, H, W = masks_tensor.shape
    fx, fy, cx, cy = (cam_K[0, 0], cam_K[1, 1], cam_K[0, 2], cam_K[1, 2])
    u, v = torch.meshgrid(
        torch.arange(0, W, device=device),
        torch.arange(0, H, device=device),
        indexing="xy",
    )
    depth = depth_tensor.repeat(N, 1, 1) * masks_tensor
    valid = (depth > 0).float()
    x = depth
    y = -(u - cx) * depth / fx
    z = -(v - cy) * depth / fy
    points = torch.stack((x, y, z), dim=-1) * valid.unsqueeze(-1)
    if image_rgb_tensor is not None:
        repeated_rgb = image_rgb_tensor.repeat(N, 1, 1, 1) * masks_tensor.unsqueeze(-1)
        colors = repeated_rgb * valid.unsqueeze(-1)
    else:
        print("No RGB image provided, assigning random colors to objects")
        logging.warning("No RGB image provided, assigning random colors to objects")
        random_colors = (
            torch.randint(0, 256, (N, 3), device=device, dtype=torch.float32) / 255.0
        )
        colors = random_colors.unsqueeze(1).unsqueeze(1).expand(
            -1, H, W, -1
        ) * valid.unsqueeze(-1)
    return (points, colors)


def detections_to_obj_pcd_and_bbox(
    depth_array,
    masks,
    cam_K,
    image_rgb=None,
    trans_pose=None,
    min_points_threshold=5,
    spatial_sim_type="axis_aligned",
    obj_pcd_max_points=None,
    downsample_voxel_size=None,
    dbscan_remove_noise=None,
    dbscan_eps=None,
    dbscan_min_points=None,
    run_dbscan=None,
    device="cuda",
):
    N, H, W = masks.shape
    if trans_pose is not None:
        trans_pose = torch.tensor(trans_pose, device=device, dtype=torch.float)
    depth_tensor = torch.from_numpy(depth_array).to(device).float()
    masks_tensor = torch.from_numpy(masks).to(device).float()
    cam_K_tensor = torch.from_numpy(cam_K).to(device).float()
    if image_rgb is not None:
        image_rgb_tensor = torch.from_numpy(image_rgb).to(device).float() / 255.0
    else:
        image_rgb_tensor = None
    points_tensor, colors_tensor = batch_mask_depth_to_points_colors(
        depth_tensor, masks_tensor, cam_K_tensor, image_rgb_tensor, device
    )
    processed_objects = [None] * N
    for i in range(N):
        mask_points = points_tensor[i]
        mask_colors = colors_tensor[i] if colors_tensor is not None else None
        valid_points_mask = mask_points[:, :, 0] > 0
        if torch.sum(valid_points_mask) < min_points_threshold:
            continue
        valid_points = mask_points[valid_points_mask]
        valid_colors = (
            mask_colors[valid_points_mask] if mask_colors is not None else None
        )
        downsampled_points, downsampled_colors = dynamic_downsample(
            valid_points, colors=valid_colors, target=obj_pcd_max_points
        )
        if trans_pose is not None:
            downsampled_points = torch.cat(
                (downsampled_points, torch.ones_like(downsampled_points[:, :1])), dim=1
            ).transpose(0, 1)
            downsampled_points = trans_pose @ downsampled_points
            downsampled_points = downsampled_points[:3].transpose(0, 1)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(downsampled_points.cpu().numpy())
        if downsampled_colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(downsampled_colors.cpu().numpy())
        bbox = get_bounding_box(spatial_sim_type, pcd)
        if bbox.volume() < 1e-06:
            continue
        processed_objects[i] = {"pcd": pcd, "bbox": bbox}
    return processed_objects


def processing_needed(
    process_interval, run_on_final_frame, frame_idx, is_final_frame=False
):
    if process_interval > 0 and (frame_idx + 1) % process_interval == 0:
        return True
    if run_on_final_frame and is_final_frame:
        return True
    return False
