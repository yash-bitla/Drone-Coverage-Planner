from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import shapely
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry


@dataclass(frozen=True, eq=False)
class Obstacle:
    geometry: BaseGeometry
    height_m: float | None = None

    def blocks(self, altitude_m: float, clearance_m: float) -> bool:
        return self.height_m is None or self.height_m + clearance_m >= altitude_m


def inflate(
    footprints: Sequence[BaseGeometry], *, clearance_m: float, simplify_m: float
) -> list[Polygon]:
    """Merge, simplify and grow blocking footprints by the clearance.

    Enclosed courtyards are filled: a drone that cannot fly over the walls cannot reach them.
    """
    if not footprints:
        return []
    merged = shapely.union_all([shapely.make_valid(g) for g in footprints]).simplify(simplify_m)
    grown = merged.buffer(clearance_m, join_style="mitre")
    filled = [Polygon(p.exterior) for p in shapely.get_parts(grown) if p.geom_type == "Polygon"]
    return [p for p in shapely.get_parts(shapely.union_all(filled)) if p.area > 0]
