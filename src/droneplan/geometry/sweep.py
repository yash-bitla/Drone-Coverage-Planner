from __future__ import annotations

import numpy as np
from shapely.geometry.base import BaseGeometry

from droneplan.errors import InvalidInputError


def min_width_sweep_angle(geom: BaseGeometry) -> float:
    """Lane direction in [0, pi) that minimises the number of lanes.

    A convex region's minimum width is attained with a hull edge flush against a supporting
    line (rotating calipers), so only hull-edge directions need testing.
    """
    hull = geom.convex_hull
    if hull.geom_type != "Polygon" or hull.area <= 0:
        raise InvalidInputError("area must have non-zero extent")
    coords = np.asarray(hull.exterior.coords)
    edges = np.diff(coords, axis=0)
    best_angle, best_width = 0.0, np.inf
    for angle in np.unique(np.mod(np.arctan2(edges[:, 1], edges[:, 0]), np.pi)):
        proj = coords @ np.array([-np.sin(angle), np.cos(angle)])
        width = proj.max() - proj.min()
        if width < best_width - 1e-9:
            best_angle, best_width = float(angle), float(width)
    return best_angle
