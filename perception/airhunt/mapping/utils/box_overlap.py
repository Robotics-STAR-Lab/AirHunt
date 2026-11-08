"""Exact convex-box overlap for mapping, without a compiled PyTorch3D dependency."""

from itertools import combinations
import numpy as np
import torch
from scipy.spatial import ConvexHull, QhullError


def _shape(points):
    hull = ConvexHull(points)
    planes = np.unique(np.round(hull.equations, 10), axis=0)
    return (planes, float(hull.volume))


def box3d_overlap(boxes1, boxes2):
    a = boxes1.detach().cpu().double().numpy()
    b = boxes2.detach().cpu().double().numpy()
    if a.shape[1:] != (8, 3) or b.shape[1:] != (8, 3):
        raise ValueError("Expected (N, 8, 3) boxes")
    volumes = np.zeros((len(a), len(b)))
    ious = np.zeros_like(volumes)

    def shapes(boxes):
        result = []
        for box in boxes:
            try:
                result.append(_shape(box))
            except QhullError:
                result.append(None)
        return result

    sa, sb = (shapes(a), shapes(b))
    for i, aa in enumerate(a):
        if sa[i] is None:
            continue
        for j, bb in enumerate(b):
            if sb[j] is None or np.any(
                np.minimum(aa.max(0), bb.max(0)) <= np.maximum(aa.min(0), bb.min(0))
            ):
                continue
            planes = np.concatenate((sa[i][0], sb[j][0]))
            triples = np.array(list(combinations(range(len(planes)), 3)))
            mat = planes[triples, :3]
            rhs = -planes[triples, 3]
            valid = np.abs(np.linalg.det(mat)) > 1e-09
            if not valid.any():
                continue
            vertices = np.linalg.solve(mat[valid], rhs[valid])
            inside = (vertices @ planes[:, :3].T + planes[:, 3] <= 1e-07).all(axis=1)
            vertices = vertices[inside]
            if len(vertices) < 4:
                continue
            try:
                inter = float(ConvexHull(vertices).volume)
            except QhullError:
                continue
            inter = min(inter, sa[i][1], sb[j][1])
            volumes[i, j] = inter
            ious[i, j] = inter / max(sa[i][1] + sb[j][1] - inter, 1e-12)
    return (
        torch.as_tensor(volumes, device=boxes1.device, dtype=boxes1.dtype),
        torch.as_tensor(ious, device=boxes1.device, dtype=boxes1.dtype),
    )
