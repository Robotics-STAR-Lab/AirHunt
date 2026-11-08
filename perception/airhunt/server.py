import os
import argparse
from matplotlib import pyplot as plt
from omegaconf import OmegaConf
import random
import numpy as np
import time
import logging
import cv2
import threading
import open_clip
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from groundingdino.util.inference import load_model
from airhunt.geometry import get_cam_intr
from airhunt.scene import Scene
from scipy.spatial.transform import Rotation as R
from airhunt.cameras import (
    AirsimDataLoader,
    get_pts_angle_airsim,
    prepare_airsim_navigation_goals,
)
from airhunt.reasoning import query_vlm_task1, query_vlm_task2
from airhunt.logging import AirsimLogger
from airhunt.bridge import pos_airsim_to_memory
import rospy


def initialize_models_and_scene(
    cfg, cfg_cg, scene_name, pts_memory, logging_path, formatter
):
    sam2_model = build_sam2(
        cfg.sam2_model_config, cfg.sam2_checkpoint, device=cfg.device
    )
    sam_predictor = SAM2ImagePredictor(sam2_model)
    logging.info(f"Load SAM2 model {cfg.sam2_checkpoint} successful!")
    detection_model = load_model(
        model_config_path=cfg.grounding_dino_config,
        model_checkpoint_path=cfg.grounding_dino_checkpoint,
        device=cfg.device,
    )
    logging.info(f"Load Grounding DINO model successful!")
    clip_model, _, clip_preprocess = open_clip.create_model_and_transforms(
        "ViT-B-32", "laion2b_s34b_b79k"
    )
    clip_tokenizer = open_clip.get_tokenizer("ViT-B-32")
    logging.info(f"Load CLIP model successful!")
    if torch.cuda.is_available() and torch.cuda.get_device_properties(0).major >= 8:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
    scene = Scene(
        scene_name,
        cfg.default_env,
        cfg,
        cfg_cg,
        detection_model,
        sam_predictor,
        clip_model,
        clip_preprocess,
        clip_tokenizer,
        pts_memory,
        logging_path,
        formatter,
    )
    return (
        scene,
        detection_model,
        sam_predictor,
        clip_model,
        clip_preprocess,
        clip_tokenizer,
    )


def setup_logging(cfg):
    cfg.output_dir = os.path.join(cfg.output_parent_dir, cfg.exp_name)
    if not os.path.exists(cfg.output_dir):
        os.makedirs(cfg.output_dir, exist_ok=True)
    logging_path = os.path.join(str(cfg.output_dir), "navigation.log")

    class ElapsedTimeFormatter(logging.Formatter):

        def __init__(self, fmt=None, datefmt=None):
            super().__init__(fmt, datefmt)
            self.start_time = time.time()

        def formatTime(self, record, datefmt=None):
            elapsed_seconds = record.created - self.start_time
            hours, remainder = divmod(elapsed_seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            return f"{int(hours):02}:{int(minutes):02}:{int(seconds):02}"

    formatter = ElapsedTimeFormatter(fmt="%(asctime)s - %(message)s")
    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s",
        handlers=[logging.FileHandler(logging_path, mode="w"), logging.StreamHandler()],
    )
    for handler in logging.getLogger().handlers:
        handler.setFormatter(formatter)
    return (logging_path, formatter)


def update_snapshots(scene, traj_flu, all_added_obj_ids):
    all_added_obj_ids = [
        obj_id for obj_id in all_added_obj_ids if obj_id in scene.objects
    ]
    for obj_id, obj in scene.objects.items():
        for traj_point in traj_flu:
            traj_point = np.array(traj_point)
            if (
                np.linalg.norm(obj["bbox"].center[[0, 1]] - traj_point[[0, 1]])
                < cfg.scene_graph.obj_include_dist + 0.5
            ):
                all_added_obj_ids.append(obj_id)
                break
    scene.update_snapshots(
        obj_ids=set(all_added_obj_ids), min_detection=cfg.min_detection
    )
    return all_added_obj_ids


def run_task1_async(
    subtask_metadata, egocentric_views, cfg, global_step, logger, subtask_id, task1_data
):
    try:
        task1_results = query_vlm_task1(
            subtask_metadata=subtask_metadata,
            egocentric_views=egocentric_views,
            cfg=cfg,
            cnt_step=global_step,
            logger=logger,
            subtask_id=subtask_id,
            verbose=True,
        )
        if task1_results:
            logging.info(
                f"Task1 VLM query successful! Got {len(task1_results)} results"
            )
            task1_data["results"] = task1_results
        else:
            raise ValueError("Task1 VLM query returned empty results")
    except Exception as e:
        raise ValueError(f"Error during Task1 VLM processing: {e}")


def run_task2_async(
    subtask_metadata, scene, cfg, global_step, logger, subtask_id, task2_data
):
    try:
        task2_results = query_vlm_task2(
            subtask_metadata=subtask_metadata,
            scene=scene,
            cfg=cfg,
            cnt_step=global_step,
            logger=logger,
            subtask_id=subtask_id,
            verbose=True,
        )
        if task2_results:
            if task2_results["max_point_choice"] is not None:
                logging.info(f"Task2 VLM query successful!")
                scene.airsim_bridge.publish_task2_data(task2_results, scene.objects)
                task2_data["published"] = True
                task2_data["results"] = task2_results
        else:
            raise ValueError("Task2 VLM query returned no valid results")
    except Exception as e:
        task2_data["error"] = str(e)
        raise ValueError(f"Error during Task2 VLM processing: {e}")


def run_navigation(cfg):
    cfg_cg = OmegaConf.load(cfg.concept_graph_config_path)
    OmegaConf.resolve(cfg_cg)
    random.seed(cfg.seed)
    np.random.seed(cfg.seed)
    logging_path, formatter = setup_logging(cfg)
    data_loader = AirsimDataLoader(cfg.dataset_path)
    scene_list = data_loader.get_scene_list()
    logger = AirsimLogger(cfg.output_dir)
    if scene_list:
        scene_name = scene_list[0]
        scene_data = data_loader.get_scene_data(scene_name)
        episodes = scene_data.get("episodes", [])
        if episodes:
            episode = episodes[0]
            pts_airsim, angle = get_pts_angle_airsim(
                episode["start_position"], episode["start_rotation"]
            )
            pts_memory = pos_airsim_to_memory(pts_airsim)
            scene, _, _, _, _, _ = initialize_models_and_scene(
                cfg, cfg_cg, scene_name, pts_memory, logging_path, formatter
            )
            episode_dir, eps_frontier_dir, eps_snapshot_dir = logger.init_episode(
                episode_id=f"{scene_name}_3"
            )
            all_subtask_goal_types, all_subtask_goals = prepare_airsim_navigation_goals(
                scene_name=scene_name,
                episode=episode,
                all_navigation_goals=episode.get("goals", []),
            )
            if all_subtask_goals:
                subtask_goal = all_subtask_goals[0]
                goal_type = all_subtask_goal_types[0]
                subtask_id = f"{scene_name}_{episode['episode_id']}_0"
                subtask_metadata = logger.init_subtask(
                    subtask_id=subtask_id,
                    goal_type=goal_type,
                    subtask_goal=subtask_goal,
                    pts=pts_memory,
                    scene=scene,
                )
                logging.info("Starting continuous camera data collection...")
                cam_intr = get_cam_intr(cfg.hfov, cfg.img_height, cfg.img_width)
                cameras_to_use = ["front"]
                global_step = 0
                all_added_obj_ids = []
                time.sleep(1)
                started = False
                first = cfg.initial_panorama
                while True:
                    global_step += 1
                    start_time = time.time()
                    max_wait_time = 30.0
                    buffer_size = 3
                    count = 0
                    while (
                        scene.airsim_bridge.fsm_state is None
                        or scene.airsim_bridge.fsm_state != "WAIT_TRIGGER"
                    ) and (not started):
                        count += 1
                        if count % 5 == 0:
                            logging.info(
                                f"Waiting for FSM state to be ready, current state: {scene.airsim_bridge.fsm_state}"
                            )
                        time.sleep(0.2)
                        continue
                    started = True
                    while True:
                        buffer_status = scene.airsim_bridge.get_buffer_status()
                        logging.info(
                            f"Buffer status: {buffer_status['size']} frames, time span: {buffer_status['time_span']:.2f}s"
                        )
                        if buffer_status["size"] >= buffer_size:
                            logging.info(
                                f"Buffer filled with {buffer_status['size']} frames"
                            )
                            break
                        if time.time() - start_time > max_wait_time:
                            logging.warning(
                                f"Timeout waiting for buffer. Using {buffer_status['size']} frames."
                            )
                            break
                        time.sleep(0.2)
                    buffered_data = scene.airsim_bridge.get_buffered_camera_data(
                        buffer_size=buffer_size, cameras=cameras_to_use, first=first
                    )
                    time.sleep(1)
                    if buffered_data:
                        first = False
                        scene.airsim_bridge.clear_camera_buffer()
                    os.makedirs("tmp_buffer", exist_ok=True)
                    for i, data_entry in enumerate(buffered_data):
                        rgb_img = data_entry["rgb"]
                        rgb_bgr = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
                        filename = f"tmp_buffer/{global_step}_buffer_{i}_{data_entry['camera_name']}.png"
                        cv2.imwrite(filename, rgb_bgr)
                        logging.info(f"保存图像: {filename}")
                    logging.info(
                        f"Retrieved {len(buffered_data)} camera data entries from buffer"
                    )
                    egocentric_views = []
                    traj_flu = []
                    batch_added_obj_ids = []
                    for idx, data_entry in enumerate(buffered_data):
                        rgb = data_entry["rgb"]
                        depth = data_entry["depth_msg"]
                        depth_cv = data_entry["depth_cv"]
                        pose_msg = data_entry["pose"]
                        camera_name = data_entry["camera_name"]
                        timestamp = data_entry["timestamp"]
                        pos_flu = [
                            pose_msg.pose.position.x,
                            pose_msg.pose.position.y,
                            pose_msg.pose.position.z,
                        ]
                        quat_flu = [
                            pose_msg.pose.orientation.x,
                            pose_msg.pose.orientation.y,
                            pose_msg.pose.orientation.z,
                            pose_msg.pose.orientation.w,
                        ]
                        R_flu = R.from_quat(quat_flu).as_matrix()
                        pose_flu = np.eye(4)
                        pose_flu[:3, :3] = R_flu
                        pose_flu[:3, 3] = pos_flu
                        yaw_flu, roll_flu, pitch_flu = R.from_matrix(R_flu).as_euler(
                            "zxy", degrees=False
                        )
                        traj_flu.append(pos_flu)
                        depth_min = 0.1
                        depth_max = float(cfg.max_dist)
                        depth_clipped = depth_cv.copy()
                        depth_clipped[depth_clipped < depth_min] = 0
                        depth_clipped[depth_clipped > depth_max] = 0
                        obs_file_name = f"{global_step}-buffer_{idx}-{camera_name}.png"
                        obs_depth_file_name = (
                            f"depth_{global_step}-buffer_{idx}-{camera_name}.png"
                        )
                        with torch.no_grad():
                            text_prompt = subtask_metadata.get("class", None)
                            if text_prompt:
                                if not text_prompt.endswith("."):
                                    text_prompt += "."
                            annotated_rgb, added_obj_ids = scene.update_scene_graph(
                                image_rgb=rgb[..., :3],
                                depth=depth_clipped,
                                intrinsics=cam_intr,
                                cam_pose=pose_flu,
                                pts=np.array(pos_flu),
                                angle=yaw_flu,
                                img_path=obs_file_name,
                                text_prompt=text_prompt,
                            )
                            batch_added_obj_ids.extend(added_obj_ids)
                            scene.all_observations[obs_file_name] = rgb
                        plt.imsave(
                            os.path.join(eps_snapshot_dir, obs_file_name), annotated_rgb
                        )
                        plt.imsave(
                            os.path.join(eps_snapshot_dir, obs_depth_file_name),
                            depth_cv,
                        )
                        egocentric_views.append(
                            {
                                "image": rgb,
                                "depth": depth,
                                "pos": pos_flu,
                                "angle": yaw_flu,
                                "pose": pose_msg,
                                "camera_name": camera_name,
                                "timestamp": timestamp,
                            }
                        )
                    logging.info(
                        f"Built {len(egocentric_views)} egocentric views from buffered data"
                    )
                    all_added_obj_ids += batch_added_obj_ids
                    update_snapshots(scene, traj_flu, all_added_obj_ids)
                    logging.info("Object memory: %s", [
                        {"id": int(obj_id), "observations": int(obj["num_detections"]),
                         "center": np.round(obj["bbox"].center, 2).tolist()}
                        for obj_id, obj in scene.objects.items()
                    ])
                    logging.info("Starting async VLM queries...")
                    if egocentric_views:
                        task2_data = {"published": False}
                        task1_data = {}
                        task1_thread = threading.Thread(
                            target=run_task1_async,
                            args=(
                                subtask_metadata,
                                egocentric_views,
                                cfg,
                                global_step,
                                logger,
                                subtask_id,
                                task1_data,
                            ),
                        )
                        task2_thread = threading.Thread(
                            target=run_task2_async,
                            args=(
                                subtask_metadata,
                                scene,
                                cfg,
                                global_step,
                                logger,
                                subtask_id,
                                task2_data,
                            ),
                        )
                        task1_thread.start()
                        task2_thread.start()
                        logging.info("Waiting for both VLM tasks to complete...")
                        task1_thread.join()
                        task2_thread.join()
                        logging.info(
                            "Both VLM tasks completed, proceeding to next iteration"
                        )
                        if task2_data["published"]:
                            logging.info(
                                f"Published task2 data: {task2_data.get('results', {}).get('max_point_choice', 'Unknown')}"
                            )
                            logging.info(
                                f"++++++++++++++++++++++++++++++++Task Finished++++++++++++++++++++++++++++++++++++++++"
                            )
                            rospy.signal_shutdown("Task completed")
                            break
                        if "results" in task1_data:
                            scene.airsim_bridge.publish_task1_data(task1_data["results"])
                        egocentric_views.clear()
                        logging.info("Camera buffer cleared after VLM query")
                    else:
                        logging.info("No egocentric views available for VLM query")
            else:
                logging.info("No subtask goals available")
            logging.info("Navigation service loop ended")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SOP navigation perception service")
    parser.add_argument("-cf", "--config", required=True)
    args = parser.parse_args()
    cfg = OmegaConf.load(args.config)
    OmegaConf.resolve(cfg)
    rospy.init_node("airhunt_perception")
    try:
        run_navigation(cfg)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("User interrupt")
