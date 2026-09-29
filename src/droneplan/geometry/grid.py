from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry

from droneplan._types import BoolArray, FloatArray, IntArray
from droneplan.errors import InvalidInputError


@dataclass(frozen=True, eq=False)
class Grid:
    """Row 0 is the top row; `origin` is the top-left corner (x_min, y_max).

    `required` holds mappable cells only; cells that overlap the area but also an obstacle are
    in `blocked` and counted in `n_unmappable`.
    """

    required: BoolArray
    cell_size: float
    origin: tuple[float, float] = (0.0, 0.0)
    blocked: BoolArray | None = None
    n_unmappable: int = 0

    def __post_init__(self) -> None:
        if self.required.ndim != 2 or self.required.dtype != np.bool_:
            raise InvalidInputError("required must be a 2-D boolean array")
        if self.cell_size <= 0:
            raise InvalidInputError("cell_size must be > 0")
        if not self.required.any():
            raise InvalidInputError("area contains no mappable cells")
        if self.blocked is None:
            object.__setattr__(self, "blocked", np.zeros_like(self.required))

    @classmethod
    def from_mask(
        cls, mask: BoolArray, cell_size: float, origin: tuple[float, float] = (0.0, 0.0)
    ) -> Grid:
        return cls(np.asarray(mask, dtype=bool), float(cell_size), origin)

    @property
    def shape(self) -> tuple[int, int]:
        return (int(self.required.shape[0]), int(self.required.shape[1]))

    @property
    def n_required(self) -> int:
        return int(self.required.sum())

    def required_cells(self) -> IntArray:
        return np.argwhere(self.required).astype(np.int64)

    def centers(self, rc: IntArray) -> FloatArray:
        rc = np.asarray(rc).reshape(-1, 2)
        x0, y0 = self.origin
        x = x0 + (rc[:, 1] + 0.5) * self.cell_size
        y = y0 - (rc[:, 0] + 0.5) * self.cell_size
        return np.column_stack([x, y]).astype(np.float64)


def _overlaps(geom: BaseGeometry, boxes: np.ndarray) -> BoolArray:
    shapely.prepare(geom)
    return np.asarray(shapely.intersects(geom, boxes) & ~shapely.touches(geom, boxes), dtype=bool)


def rasterize(
    area: BaseGeometry,
    cell_size: float,
    *,
    max_cells: int,
    obstacles: Sequence[Polygon] = (),
) -> Grid:
    if area.is_empty or area.area <= 0:
        raise InvalidInputError("area is empty")
    minx, miny, maxx, maxy = area.bounds
    cols = max(1, math.ceil((maxx - minx) / cell_size - 1e-9))
    rows = max(1, math.ceil((maxy - miny) / cell_size - 1e-9))
    if rows * cols > max_cells:
        raise InvalidInputError(
            f"grid of {rows}x{cols} cells exceeds max_cells={max_cells}; increase the swath width"
        )
    left, top = np.meshgrid(minx + np.arange(cols) * cell_size, maxy - np.arange(rows) * cell_size)
    boxes = shapely.box(left, top - cell_size, left + cell_size, top)
    in_area = _overlaps(area, boxes)
    blocked = (
        _overlaps(shapely.union_all(list(obstacles)), boxes)
        if obstacles
        else np.zeros_like(in_area)
    )
    return Grid(
        required=in_area & ~blocked,
        cell_size=float(cell_size),
        origin=(float(minx), float(maxy)),
        blocked=blocked,
        n_unmappable=int((in_area & blocked).sum()),
    )
