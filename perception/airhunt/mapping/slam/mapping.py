import torch
import torch.nn.functional as F
from typing import List, Optional, Tuple
from airhunt.mapping.slam.slam_classes import MapObjectDict, DetectionDict
from airhunt.mapping.utils.ious import (
    compute_iou_batch,
    compute_giou_batch,
    compute_3d_iou_accurate_batch,
    compute_3d_giou_accurate_batch,
)
from airhunt.mapping.slam.utils import compute_overlap_matrix_general


def compute_spatial_similarities(
    spatial_sim_type: str,
    detection_list: DetectionDict,
    objects: MapObjectDict,
    downsample_voxel_size,
) -> torch.Tensor:
    det_bboxes = detection_list.get_stacked_values_torch("bbox")
    obj_bboxes = objects.get_stacked_values_torch("bbox")
    if spatial_sim_type == "iou":
        spatial_sim = compute_iou_batch(det_bboxes, obj_bboxes)
    elif spatial_sim_type == "giou":
        spatial_sim = compute_giou_batch(det_bboxes, obj_bboxes)
    elif spatial_sim_type == "iou_accurate":
        spatial_sim = compute_3d_iou_accurate_batch(det_bboxes, obj_bboxes)
    elif spatial_sim_type == "giou_accurate":
        spatial_sim = compute_3d_giou_accurate_batch(det_bboxes, obj_bboxes)
    elif spatial_sim_type == "overlap":
        spatial_sim = compute_overlap_matrix_general(
            objects, detection_list, downsample_voxel_size
        )
        spatial_sim = torch.from_numpy(spatial_sim).T
    else:
        raise ValueError(f"Invalid spatial similarity type: {spatial_sim_type}")
    return spatial_sim


def compute_visual_similarities(
    detection_list: DetectionDict, objects: MapObjectDict
) -> torch.Tensor:
    det_fts = detection_list.get_stacked_values_torch("clip_ft")
    obj_fts = objects.get_stacked_values_torch("clip_ft")
    det_fts = det_fts.unsqueeze(-1)
    obj_fts = obj_fts.T.unsqueeze(0)
    visual_sim = F.cosine_similarity(det_fts, obj_fts, dim=1)
    return visual_sim


def aggregate_similarities(
    match_method: str,
    phys_bias: float,
    spatial_sim: torch.Tensor,
    visual_sim: torch.Tensor,
) -> torch.Tensor:
    if match_method == "sim_sum":
        sims = (1 + phys_bias) * spatial_sim + (1 - phys_bias) * visual_sim
    else:
        raise ValueError(f"Unknown matching method: {match_method}")
    return sims


def match_detections_to_objects(
    agg_sim: torch.Tensor,
    existing_obj_ids: List[int],
    detected_obj_ids: List[int],
    detection_threshold: float = float("-inf"),
) -> List[Tuple[int, Optional[int]]]:
    match_indices = []
    for detected_obj_idx in range(agg_sim.shape[0]):
        max_sim_value = agg_sim[detected_obj_idx].max()
        if max_sim_value <= detection_threshold:
            match_indices.append((detected_obj_ids[detected_obj_idx], None))
        else:
            match_indices.append(
                (
                    detected_obj_ids[detected_obj_idx],
                    existing_obj_ids[agg_sim[detected_obj_idx].argmax().item()],
                )
            )
    return match_indices
