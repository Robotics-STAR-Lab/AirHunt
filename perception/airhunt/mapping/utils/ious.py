import numpy as np
import torch
import open3d as o3d


def compute_iou_batch(bbox1: torch.Tensor, bbox2: torch.Tensor) -> torch.Tensor:
    bbox1_min, _ = bbox1.min(dim=1)
    bbox1_max, _ = bbox1.max(dim=1)
    bbox2_min, _ = bbox2.min(dim=1)
    bbox2_max, _ = bbox2.max(dim=1)
    bbox1_min = bbox1_min.unsqueeze(1)
    bbox1_max = bbox1_max.unsqueeze(1)
    bbox2_min = bbox2_min.unsqueeze(0)
    bbox2_max = bbox2_max.unsqueeze(0)
    inter_min = torch.max(bbox1_min, bbox2_min)
    inter_max = torch.min(bbox1_max, bbox2_max)
    inter_vol = torch.prod(torch.clamp(inter_max - inter_min, min=0), dim=2)
    bbox1_vol = torch.prod(bbox1_max - bbox1_min, dim=2)
    bbox2_vol = torch.prod(bbox2_max - bbox2_min, dim=2)
    iou = inter_vol / (bbox1_vol + bbox2_vol - inter_vol + 1e-10)
    return iou


def compute_giou_batch(bbox1: torch.Tensor, bbox2: torch.Tensor) -> torch.Tensor:
    bbox1_min, _ = bbox1.min(dim=1)
    bbox1_max, _ = bbox1.max(dim=1)
    bbox2_min, _ = bbox2.min(dim=1)
    bbox2_max, _ = bbox2.max(dim=1)
    bbox1_min = bbox1_min.unsqueeze(1)
    bbox1_max = bbox1_max.unsqueeze(1)
    bbox2_min = bbox2_min.unsqueeze(0)
    bbox2_max = bbox2_max.unsqueeze(0)
    inter_min = torch.max(bbox1_min, bbox2_min)
    inter_max = torch.min(bbox1_max, bbox2_max)
    enclosing_min = torch.min(bbox1_min, bbox2_min)
    enclosing_max = torch.max(bbox1_max, bbox2_max)
    inter_vol = torch.prod(torch.clamp(inter_max - inter_min, min=0), dim=2)
    enclosing_vol = torch.prod(enclosing_max - enclosing_min, dim=2)
    bbox1_vol = torch.prod(bbox1_max - bbox1_min, dim=2)
    bbox2_vol = torch.prod(bbox2_max - bbox2_min, dim=2)
    union_vol = bbox1_vol + bbox2_vol - inter_vol
    iou = inter_vol / (union_vol + 1e-10)
    giou = iou - (enclosing_vol - union_vol) / (enclosing_vol + 1e-10)
    return giou


def compute_3d_iou_accurate_batch(bbox1, bbox2):
    bbox1 = expand_3d_box(bbox1, 0.02)
    bbox2 = expand_3d_box(bbox2, 0.02)
    from airhunt.mapping.utils.box_overlap import box3d_overlap

    bbox1 = bbox1[:, [0, 2, 5, 3, 1, 7, 4, 6]]
    bbox2 = bbox2[:, [0, 2, 5, 3, 1, 7, 4, 6]]
    inter_vol, iou = box3d_overlap(bbox1.float(), bbox2.float())
    return iou


def compute_3d_box_volume_batch(bbox: torch.Tensor) -> torch.Tensor:
    a = torch.linalg.vector_norm(bbox[:, 0, :] - bbox[:, 1, :], ord=2, dim=1)
    b = torch.linalg.vector_norm(bbox[:, 0, :] - bbox[:, 2, :], ord=2, dim=1)
    c = torch.linalg.vector_norm(bbox[:, 0, :] - bbox[:, 3, :], ord=2, dim=1)
    vol = a * b * c
    return vol


def expand_3d_box(bbox: torch.Tensor, eps=0.02) -> torch.Tensor:
    center = bbox.mean(dim=1)
    va = bbox[:, 1, :] - bbox[:, 0, :]
    vb = bbox[:, 2, :] - bbox[:, 0, :]
    vc = bbox[:, 3, :] - bbox[:, 0, :]
    a = torch.linalg.vector_norm(va, ord=2, dim=1, keepdim=True)
    b = torch.linalg.vector_norm(vb, ord=2, dim=1, keepdim=True)
    c = torch.linalg.vector_norm(vc, ord=2, dim=1, keepdim=True)
    va = torch.where(a < eps, va / a * eps, va)
    vb = torch.where(b < eps, vb / b * eps, vb)
    vc = torch.where(c < eps, vc / c * eps, vc)
    new_bbox = torch.stack(
        [
            center - va / 2.0 - vb / 2.0 - vc / 2.0,
            center + va / 2.0 - vb / 2.0 - vc / 2.0,
            center - va / 2.0 + vb / 2.0 - vc / 2.0,
            center - va / 2.0 - vb / 2.0 + vc / 2.0,
            center + va / 2.0 + vb / 2.0 + vc / 2.0,
            center - va / 2.0 + vb / 2.0 + vc / 2.0,
            center + va / 2.0 - vb / 2.0 + vc / 2.0,
            center + va / 2.0 + vb / 2.0 - vc / 2.0,
        ],
        dim=1,
    )
    new_bbox = new_bbox.to(bbox.device)
    new_bbox = new_bbox.type(bbox.dtype)
    return new_bbox


def compute_enclosing_vol(bbox1: torch.Tensor, bbox2: torch.Tensor) -> torch.Tensor:
    M = bbox1.shape[0]
    N = bbox2.shape[0]
    enclosing_vol = torch.zeros((M, N), dtype=bbox1.dtype, device=bbox1.device)
    for i in range(bbox1.shape[0]):
        for j in range(bbox2.shape[0]):
            pcd_union = o3d.geometry.PointCloud()
            bbox_points_union = torch.cat([bbox1[i], bbox2[j]], dim=0)
            pcd_union.points = o3d.utility.Vector3dVector(
                bbox_points_union.cpu().numpy()
            )
            enclosing_mesh, _ = pcd_union.compute_convex_hull(joggle_inputs=True)
            try:
                enclosing_vol[i, j] = enclosing_mesh.get_volume()
            except:
                enclosing_mesh = pcd_union.get_axis_aligned_bounding_box()
                enclosing_vol[i, j] = enclosing_mesh.volume()
    return enclosing_vol


def compute_3d_giou_accurate_batch(
    bbox1: torch.Tensor, bbox2: torch.Tensor
) -> torch.Tensor:
    bbox1 = expand_3d_box(bbox1, 0.02)
    bbox2 = expand_3d_box(bbox2, 0.02)
    bbox1_vol = compute_3d_box_volume_batch(bbox1)
    bbox2_vol = compute_3d_box_volume_batch(bbox2)
    from airhunt.mapping.utils.box_overlap import box3d_overlap

    inter_vol, iou = box3d_overlap(
        bbox1[:, [0, 2, 5, 3, 1, 7, 4, 6]].float(),
        bbox2[:, [0, 2, 5, 3, 1, 7, 4, 6]].float(),
    )
    union_vol = bbox1_vol.unsqueeze(1) + bbox2_vol.unsqueeze(0) - inter_vol
    enclosing_vol = compute_enclosing_vol(bbox1, bbox2)
    giou = iou - (enclosing_vol - union_vol) / enclosing_vol
    return giou


def mask_subtract_contained(xyxy: np.ndarray, mask: np.ndarray, th1=0.8, th2=0.7):
    N = xyxy.shape[0]
    areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
    lt = np.maximum(xyxy[:, None, :2], xyxy[None, :, :2])
    rb = np.minimum(xyxy[:, None, 2:], xyxy[None, :, 2:])
    inter = (rb - lt).clip(min=0)
    inter_areas = inter[:, :, 0] * inter[:, :, 1]
    inter_over_box1 = inter_areas / areas[:, None]
    inter_over_box2 = inter_over_box1.T
    contained = (inter_over_box1 < th2) & (inter_over_box2 > th1)
    contained_idx = contained.nonzero()
    mask_sub = mask.copy()
    for i in range(len(contained_idx[0])):
        mask_sub[contained_idx[0][i]] = (
            mask_sub[contained_idx[0][i]] & ~mask_sub[contained_idx[1][i]]
        )
    return mask_sub
