import logging
import random
import numpy as np
import cv2
import os
from airhunt.qwen import task1_explore, task2_explore
from airhunt.images import resize_image
from airhunt.memory import SnapShot
from airhunt.scene import Scene


def query_vlm_task1(
    subtask_metadata: dict,
    egocentric_views: list,
    cfg,
    cnt_step,
    logger=None,
    subtask_id=None,
    verbose: bool = False,
):
    step_dict = {}
    step_dict["egocentric_imgs"] = {}
    for ego_id, egocentric_view in enumerate(egocentric_views):
        resized_rgb = resize_image(egocentric_view["image"], cfg.prompt_h, cfg.prompt_w)
        step_dict["egocentric_imgs"][ego_id] = {
            "full_img": resized_rgb,
            "depth": egocentric_view["depth"],
            "pos": egocentric_view["pos"],
            "angle": egocentric_view["angle"],
            "pose": egocentric_view["pose"],
            "sim": 0.0,
            "type": "egocentric",
        }
    step_dict["question"] = subtask_metadata["question"]
    step_dict["task_type"] = subtask_metadata["task_type"]
    step_dict["class"] = subtask_metadata["class"]
    task1_scores, task1_reason = task1_explore(
        step_dict,
        cfg,
        verbose=verbose,
        logger=logger,
        subtask_id=subtask_id,
        cnt_step=cnt_step,
    )
    logging.info(f"task1_score\n {task1_scores}")
    logging.info(f"task1_reason\n {task1_reason}")
    task1_results = []
    if task1_scores is not None and len(egocentric_views) > 0:
        os.makedirs("tmp_value", exist_ok=True)
        for i, (score, ego_view) in enumerate(zip(task1_scores, egocentric_views)):
            rgb_image = ego_view["image"]
            if isinstance(rgb_image, np.ndarray):
                bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                color = (0, 255, 0)
                thickness = 2
                text = f"Score: {score:.3f}"
                (text_width, text_height), _ = cv2.getTextSize(
                    text, font, font_scale, thickness
                )
                cv2.putText(
                    bgr_image, text, (10, 30), font, font_scale, color, thickness
                )
                filename = f"tmp_value/global_step_{cnt_step}_ego_view_{i}_score_{score:.3f}.png"
                cv2.imwrite(filename, bgr_image)
            task1_item = {
                "image": ego_view["image"],
                "depth": ego_view["depth"],
                "pos": ego_view["pos"],
                "pose": ego_view["pose"],
                "score": score,
                "reason": task1_reason[i] if i < len(task1_reason) else "",
                "type": "egocentric",
                "timestamp": ego_view["timestamp"],
            }
            task1_results.append(task1_item)
    return task1_results


def query_vlm_task2(
    subtask_metadata: dict,
    scene: Scene,
    cfg,
    cnt_step,
    logger=None,
    subtask_id=None,
    verbose: bool = False,
):
    step_dict = {}
    object_id_to_name = {
        obj_id: obj["class_name"] for obj_id, obj in scene.objects.items()
    }
    step_dict["obj_map"] = object_id_to_name
    step_dict["snapshot_objects"] = {}
    step_dict["snapshot_imgs"] = {}
    step_dict["use_full_obj_list"] = cfg.use_full_obj_list
    logging.info(f"Preparing step info for vlm ")
    for rgb_id, snapshot in scene.snapshots.items():
        if not snapshot.queried:
            snapshot.queried = True
            resized_rgb = resize_image(
                scene.all_observations[rgb_id], cfg.prompt_h, cfg.prompt_w
            )
            step_dict["snapshot_objects"][rgb_id] = snapshot.cluster
            step_dict["snapshot_imgs"][rgb_id] = {
                "full_img": resized_rgb,
                "object_crop": [],
                "rgb_id": rgb_id,
                "depth": snapshot.depth,
                "pos": snapshot.pos,
                "angle": snapshot.angle,
                "pose": snapshot.pose,
                "sim": 0.0,
                "type": "snapshot",
            }
            if cfg.use_full_obj_list:
                selected_bbox_idx = [
                    idx
                    for idx in range(len(snapshot.visual_prompt))
                    if snapshot.visual_prompt[idx].data["obj_id"][0]
                    in snapshot.full_obj_list.keys()
                ]
            else:
                selected_bbox_idx = [
                    idx
                    for idx in range(len(snapshot.visual_prompt))
                    if snapshot.visual_prompt[idx].data["obj_id"][0] in snapshot.cluster
                ]
            selected_bbox = snapshot.visual_prompt[selected_bbox_idx].xyxy.copy()
            selected_obj_ids = [
                snapshot.visual_prompt[idx].data["obj_id"][0]
                for idx in selected_bbox_idx
            ]
            H, W = scene.all_observations[rgb_id].shape[:2]
            scale_h, scale_w = (cfg.prompt_h / H, cfg.prompt_w / W)
            selected_bbox[:, 0] *= scale_w
            selected_bbox[:, 2] *= scale_w
            selected_bbox[:, 1] *= scale_h
            selected_bbox[:, 3] *= scale_h
            selected_bbox = selected_bbox.astype(int)
            for obj_id, bbox in zip(selected_obj_ids, selected_bbox):
                x1, y1, x2, y2 = bbox
                pad = max(4, int(0.25 * max(x2 - x1, y2 - y1)))
                left, top = max(0, x1 - pad), max(0, y1 - pad)
                right, bottom = min(cfg.prompt_w, x2 + pad), min(cfg.prompt_h, y2 + pad)
                step_dict["snapshot_imgs"][rgb_id]["object_crop"].append(
                    {
                        "obj_class": object_id_to_name[obj_id],
                        "obj_id": obj_id,
                        "crop": resized_rgb[top:bottom, left:right],
                        "bbox": [int(v) for v in bbox],
                    }
                )
    step_dict["question"] = subtask_metadata["question"]
    step_dict["task_type"] = subtask_metadata["task_type"]
    step_dict["class"] = subtask_metadata["class"]
    task2_result, task2_reason, snapshot_id_mapping, snapshot_crop_mapping = (
        task2_explore(
            step_dict,
            cfg,
            verbose=verbose,
            logger=logger,
            subtask_id=subtask_id,
            cnt_step=cnt_step,
        )
    )
    logging.info(f"task2_result\n {task2_result}")
    logging.info(f"task2_reason\n {task2_reason}")
    task2_results = {"reason": task2_reason, "choice": None, "max_point_choice": None}
    if task2_result is not None and task2_result != "no snapshot interested":
        if isinstance(task2_result, tuple) and len(task2_result) == 2:
            snapshot_idx, object_idx = task2_result
            snapshot_ids = list(scene.snapshots.keys())
            snapshot_id = snapshot_id_mapping[int(snapshot_idx)]
            if snapshot_id in snapshot_ids:
                pred_target_snapshot = scene.snapshots[snapshot_id]
                pred_target_snapshot_id = snapshot_id
                object_idx = snapshot_crop_mapping[pred_target_snapshot_id][object_idx]
                if object_idx < len(
                    step_dict["snapshot_imgs"][pred_target_snapshot_id]["object_crop"]
                ):
                    pred_target_obj_id = step_dict["snapshot_imgs"][
                        pred_target_snapshot_id
                    ]["object_crop"][object_idx]["obj_id"]
                    logging.info(
                        f"Next choice Object: {pred_target_obj_id}, {scene.objects[pred_target_obj_id]['class_name']}"
                    )
                    max_point_choice = SnapShot(
                        image=pred_target_snapshot.image,
                        color=(random.random(), random.random(), random.random()),
                        pos=np.empty(3),
                        full_obj_list={
                            pred_target_obj_id: scene.objects[pred_target_obj_id][
                                "conf"
                            ]
                        },
                        cluster=[pred_target_obj_id],
                        depth=None,
                        angle=None,
                    )
                    if logger is not None:
                        logger.save_max_point_choice_visualization(
                            global_step=cnt_step,
                            subtask_id=subtask_id,
                            max_point_choice=max_point_choice,
                            scene=scene,
                            object_id_to_name=object_id_to_name,
                            step_dict=step_dict,
                        )
                    task2_results["choice"] = task2_result
                    task2_results["max_point_choice"] = max_point_choice
    return task2_results
