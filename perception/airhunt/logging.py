import base64
from io import BytesIO
from PIL import Image
import os
import logging
import numpy as np
import matplotlib.pyplot as plt
import cv2
from airhunt.scene import Scene


class AirsimLogger:

    def __init__(self, output_dir):
        self.output_dir = output_dir

    def init_episode(self, episode_id):
        self.episode_dir = os.path.join(self.output_dir, episode_id)
        eps_frontier_dir = os.path.join(self.episode_dir, "frontier")
        eps_snapshot_dir = os.path.join(self.episode_dir, "snapshot")
        eps_value_map_dir = os.path.join(self.episode_dir, "value_map")
        os.makedirs(self.episode_dir, exist_ok=True)
        os.makedirs(eps_frontier_dir, exist_ok=True)
        os.makedirs(eps_snapshot_dir, exist_ok=True)
        os.makedirs(eps_value_map_dir, exist_ok=True)
        return (self.episode_dir, eps_frontier_dir, eps_snapshot_dir)

    def init_subtask(self, subtask_id, goal_type, subtask_goal, pts, scene: Scene):
        goal_category = subtask_goal[0]["object_category"]
        goal_query = subtask_goal[0]["query"]
        goal_obj_ids = [x["object_id"] for x in subtask_goal]
        from airhunt.labels import logger_object_id

        goal_obj_ids = [logger_object_id(x) for x in goal_obj_ids]
        if goal_type != "object":
            assert len(goal_obj_ids) == 1, f"{len(goal_obj_ids)} != 1"
        goal_positions = [x["position"] for x in subtask_goal]
        viewpoints = [
            view_point["agent_state"]["position"]
            for goal in subtask_goal
            for view_point in goal["view_points"]
        ]
        all_distances = []
        for viewpoint in viewpoints:
            distance = np.linalg.norm(np.array(pts) - np.array(viewpoint))
            all_distances.append(distance)
        gt_subtask_explore_dist = min(all_distances) + 1e-06 if all_distances else 1.0
        self.subtask_object_observe_dir = os.path.join(
            self.episode_dir, "object_observations"
        )
        if os.path.exists(self.subtask_object_observe_dir):
            os.system(f"rm -r {self.subtask_object_observe_dir}")
        os.makedirs(self.subtask_object_observe_dir, exist_ok=False)
        subtask_metadata = {
            "question_id": subtask_id,
            "question": None,
            "image": None,
            "answer": goal_category,
            "goal_obj_ids": goal_obj_ids,
            "class": goal_category,
            "task_type": goal_type,
            "viewpoints": viewpoints,
            "gt_subtask_explore_dist": gt_subtask_explore_dist,
        }
        if goal_type == "object":
            subtask_metadata["question"] = f"{goal_query}"
        elif goal_type == "description":
            subtask_metadata["question"] = (
                f"Could you find the object exactly described as the '{subtask_goal[0].get('lang_desc', goal_category)}'?"
            )
        else:
            subtask_metadata["question"] = (
                f"Could you find the exact object captured at the center of the following image? You need to pay attention to the environment and find the exact object."
            )
            view_pos_dict = subtask_goal[0]["view_points"][0]["agent_state"]
            obs, _ = scene.get_observation(
                pts=view_pos_dict["position"], rotation=view_pos_dict["rotation"]
            )
            plt.imsave(
                os.path.join(self.output_dir, subtask_id, "image_goal.png"),
                obs["color_sensor"],
            )
            subtask_metadata["image"] = f"{self.output_dir}/{subtask_id}/image_goal.png"
        self.pts_voxels = np.empty((0, 2))
        self.subtask_explore_dist = 0.0
        return subtask_metadata

    def save_max_point_choice_visualization(
        self,
        global_step,
        subtask_id,
        max_point_choice,
        scene,
        object_id_to_name,
        step_dict=None,
    ):
        if max_point_choice is None:
            logging.info(f"No max_point_choice to visualize for step {global_step}")
            return
        assert self.episode_dir is not None
        max_choice_vis_path = os.path.join(
            self.episode_dir, "max_point_choice_visualization"
        )
        os.makedirs(max_choice_vis_path, exist_ok=True)
        rgb_id = max_point_choice.image
        if rgb_id not in scene.all_observations:
            logging.warning(f"RGB image {rgb_id} not found in scene observations")
            return
        full_rgb = scene.all_observations[rgb_id]
        if not max_point_choice.cluster:
            logging.warning("No objects in max_point_choice cluster")
            return
        selected_obj_id = max_point_choice.cluster[0]
        selected_crop = None
        if step_dict and rgb_id in step_dict.get("snapshot_imgs", {}):
            for crop_info in step_dict["snapshot_imgs"][rgb_id]["object_crop"]:
                if crop_info["obj_id"] == selected_obj_id:
                    selected_crop = crop_info["crop"]
                    break
        selected_bbox = None
        if step_dict is None:
            original_snapshot = None
            for snapshot in scene.snapshots.values():
                if snapshot.image == rgb_id:
                    original_snapshot = snapshot
                    break
            if original_snapshot and hasattr(original_snapshot, "visual_prompt"):
                for prompt in original_snapshot.visual_prompt:
                    if hasattr(prompt, "data") and "obj_id" in prompt.data:
                        if prompt.data["obj_id"][0] == selected_obj_id:
                            selected_bbox = prompt.xyxy.copy()
                            break
        fig, axs = plt.subplots(1, 2, figsize=(16, 8))
        axs[0].imshow(full_rgb)
        axs[0].set_title(f"Full RGB Image", fontsize=14, fontweight="bold")
        axs[0].axis("off")
        if selected_crop is not None:
            axs[1].imshow(selected_crop)
            logging.info(
                f"Using processed crop from step_dict for object {selected_obj_id}"
            )
        elif selected_bbox is not None:
            from matplotlib.patches import Rectangle

            rect = Rectangle(
                (selected_bbox[0], selected_bbox[1]),
                selected_bbox[2] - selected_bbox[0],
                selected_bbox[3] - selected_bbox[1],
                linewidth=3,
                edgecolor="red",
                facecolor="none",
            )
            axs[0].add_patch(rect)
            crop = full_rgb[
                int(selected_bbox[1]) : int(selected_bbox[3]),
                int(selected_bbox[0]) : int(selected_bbox[2]),
            ]
            if crop.size > 0:
                axs[1].imshow(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                logging.info(f"Using bounding box crop for object {selected_obj_id}")
            else:
                axs[1].text(
                    0.5,
                    0.5,
                    "Invalid Crop",
                    ha="center",
                    va="center",
                    transform=axs[1].transAxes,
                )
        else:
            axs[1].text(
                0.5,
                0.5,
                "No Crop Available",
                ha="center",
                va="center",
                transform=axs[1].transAxes,
            )
            logging.warning(
                f"No crop or bounding box found for object {selected_obj_id}"
            )
        obj_class = object_id_to_name.get(selected_obj_id, "unknown")
        axs[1].set_title(
            f"Selected Object: {obj_class}\n(ID: {selected_obj_id})",
            fontsize=14,
            fontweight="bold",
        )
        axs[1].axis("off")
        fig.suptitle(
            f"Max Point Choice - Step {global_step}", fontsize=18, fontweight="bold"
        )
        plt.tight_layout()
        output_path = os.path.join(
            max_choice_vis_path, f"{global_step}_{subtask_id}_max_choice.png"
        )
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close()
        logging.info(f"Saved max_point_choice visualization: {output_path}")

    def save_snapshot_visualization(
        self, global_step, subtask_id, snapshot_imgs, snapshot_classes, snapshot_crops
    ):
        assert self.episode_dir is not None

        def decode_base64_image(base64_str):
            try:
                image_data = base64.b64decode(base64_str)
                image = Image.open(BytesIO(image_data))
                if image.mode != "RGB":
                    image = image.convert("RGB")
                return np.array(image)
            except Exception as e:
                logging.warning(f"Failed to decode image: {e}")
                return None

        snapshot_vis_path = os.path.join(self.episode_dir, "snapshot_visualization")
        os.makedirs(snapshot_vis_path, exist_ok=True)
        total_images = len(snapshot_imgs)
        for rgb_id in snapshot_crops:
            total_images += len(snapshot_crops[rgb_id])
        if total_images == 0:
            logging.info(f"No snapshots to visualize for step {global_step}")
            return
        side_length = int(np.sqrt(total_images)) + 1
        side_length = max(2, side_length)
        fig, axs = plt.subplots(side_length, side_length, figsize=(20, 20))
        if side_length == 1:
            axs = [axs]
        else:
            axs = axs.flatten()
        for ax in axs:
            ax.axis("off")
        img_idx = 0
        for i, rgb_id in enumerate(snapshot_imgs.keys()):
            if img_idx >= len(axs):
                break
            snapshot_img = decode_base64_image(snapshot_imgs[rgb_id])
            if snapshot_img is not None:
                axs[img_idx].imshow(snapshot_img)
                title = f"Snapshot {i}"
                if rgb_id in snapshot_classes and snapshot_classes[rgb_id]:
                    classes_str = ", ".join(snapshot_classes[rgb_id][:2])
                    if len(snapshot_classes[rgb_id]) > 2:
                        classes_str += "..."
                    title += f"\n({classes_str})"
                axs[img_idx].set_title(title, fontsize=16, fontweight="bold")
                img_idx += 1
            if rgb_id in snapshot_crops:
                for j, crop_base64 in enumerate(snapshot_crops[rgb_id]):
                    if img_idx >= len(axs):
                        break
                    crop_img = decode_base64_image(crop_base64)
                    if crop_img is not None:
                        axs[img_idx].imshow(crop_img)
                        class_name = "unknown"
                        if rgb_id in snapshot_classes and j < len(
                            snapshot_classes[rgb_id]
                        ):
                            class_name = snapshot_classes[rgb_id][j]
                        axs[img_idx].set_title(f"Object {j}\n{class_name}", fontsize=12)
                        img_idx += 1
        fig.suptitle(
            f"VLM Snapshot Input - Step {global_step}", fontsize=24, fontweight="bold"
        )
        plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
        output_path = os.path.join(
            snapshot_vis_path, f"{global_step}_{subtask_id}_snapshots.png"
        )
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close()
        logging.info(
            f"Saved snapshot visualization: {output_path} ({total_images} images)"
        )
