from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt
import shapely
from pyproj import CRS, Transformer
from shapely.geometry.base import BaseGeometry

from droneplan._types import FloatArray, as_xy
from droneplan.errors import InvalidInputError

_WGS84 = CRS.from_epsg(4326)


def utm_crs_for(lon: float, lat: float) -> CRS:
    if not (-180.0 <= lon <= 180.0 and -80.0 <= lat <= 84.0):
        raise InvalidInputError(f"({lon}, {lat}) is outside UTM coverage")
    zone = min(int((lon + 180.0) // 6) + 1, 60)
    return CRS.from_epsg((32600 if lat >= 0 else 32700) + zone)


class Frame:
    """lon/lat <-> plan frame: UTM metres rotated by -angle about `pivot`, so lanes run along +x."""

    def __init__(
        self, crs: CRS, angle_rad: float = 0.0, pivot: tuple[float, float] = (0.0, 0.0)
    ) -> None:
        self.crs = crs
        self.angle_rad = float(angle_rad)
        self.pivot = (float(pivot[0]), float(pivot[1]))
        self._to_utm = Transformer.from_crs(_WGS84, crs, always_xy=True)
        self._to_wgs = Transformer.from_crs(crs, _WGS84, always_xy=True)

    @classmethod
    def for_geometry(cls, geom_lnglat: BaseGeometry) -> Frame:
        c = geom_lnglat.centroid
        return cls(utm_crs_for(c.x, c.y))

    def rotated(self, angle_rad: float, pivot: tuple[float, float]) -> Frame:
        return Frame(self.crs, angle_rad, pivot)

    def lnglat_to_utm(self, lnglat: npt.ArrayLike) -> FloatArray:
        a = as_xy(lnglat)
        x, y = self._to_utm.transform(a[:, 0], a[:, 1])
        return np.column_stack([x, y])

    def utm_to_lnglat(self, xy: npt.ArrayLike) -> FloatArray:
        a = as_xy(xy)
        lng, lat = self._to_wgs.transform(a[:, 0], a[:, 1])
        return np.column_stack([lng, lat])

    def to_plan(self, lnglat: npt.ArrayLike) -> FloatArray:
        return self._rotate(self.lnglat_to_utm(lnglat), -self.angle_rad)

    def to_lnglat(self, xy: npt.ArrayLike) -> FloatArray:
        return self.utm_to_lnglat(self._rotate(as_xy(xy), self.angle_rad))

    def geometry_to_utm(self, geom: BaseGeometry) -> BaseGeometry:
        return shapely.transform(geom, self.lnglat_to_utm)

    def geometry_to_plan(self, geom: BaseGeometry) -> BaseGeometry:
        return shapely.transform(geom, self.to_plan)

    def _rotate(self, xy: FloatArray, theta: float) -> FloatArray:
        c, s = math.cos(theta), math.sin(theta)
        px, py = self.pivot
        x, y = xy[:, 0] - px, xy[:, 1] - py
        return np.column_stack([c * x - s * y + px, s * x + c * y + py])
