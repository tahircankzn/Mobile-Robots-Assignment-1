"""
planners/path_smoother.py
Catmull-Rom spline ile grid yolunu yumuşatma.

Kullanım:
    from planners.path_smoother import smooth_path
    world_pts = smooth_path(path_grid, grid_map)
"""

import numpy as np
from typing import List, Tuple


def smooth_path(
    path_grid: List[Tuple[int, int]],
    grid_map,
    pts_per_seg: int = 8,
) -> List[Tuple[float, float]]:
    """
    Grid (r, c) yolunu Catmull-Rom spline ile yumuşatarak
    dünya koordinatlarında yoğun bir yol döndürür.

    Args:
        path_grid:   (r, c) çiftleri listesi
        grid_map:    GridMap — grid_to_world() metodu gerekli
        pts_per_seg: Her grid segmenti için enterpolasyon noktası sayısı

    Returns:
        [(x, y), ...] yumuşatılmış dünya koordinatları
    """
    if not path_grid:
        return []
    if len(path_grid) == 1:
        r, c = path_grid[0]
        x, y = grid_map.grid_to_world(r, c)
        return [(x, y)]

    # Grid → dünya koordinatları
    raw: List[Tuple[float, float]] = []
    for r, c in path_grid:
        x, y = grid_map.grid_to_world(r, c)
        raw.append((x, y))

    # Ardışık tekrarları kaldır
    pts: List[Tuple[float, float]] = [raw[0]]
    for p in raw[1:]:
        if abs(p[0] - pts[-1][0]) + abs(p[1] - pts[-1][1]) > 1e-9:
            pts.append(p)

    if len(pts) < 2:
        return pts

    # Hayalet uç noktalar — yol başı/sonunda teğet sürekliliği
    g0 = (2 * pts[0][0] - pts[1][0],  2 * pts[0][1] - pts[1][1])
    gN = (2 * pts[-1][0] - pts[-2][0], 2 * pts[-1][1] - pts[-2][1])
    ctrl = [g0] + pts + [gN]

    result: List[Tuple[float, float]] = []
    for i in range(1, len(ctrl) - 2):
        p0, p1, p2, p3 = ctrl[i - 1], ctrl[i], ctrl[i + 1], ctrl[i + 2]
        for j in range(pts_per_seg):
            t = j / pts_per_seg
            x = _cr(p0[0], p1[0], p2[0], p3[0], t)
            y = _cr(p0[1], p1[1], p2[1], p3[1], t)
            result.append((x, y))

    result.append(pts[-1])   # son nokta
    return result


def _cr(p0: float, p1: float, p2: float, p3: float, t: float) -> float:
    """Catmull-Rom tek boyut enterpolasyonu (alpha = 0.5)."""
    t2 = t * t
    t3 = t2 * t
    return 0.5 * (
        (2 * p1)
        + (-p0 + p2) * t
        + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
        + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
    )
