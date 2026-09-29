from __future__ import annotations

from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
import shapely
from shapely.geometry.base import BaseGeometry

from droneplan._types import FloatArray
from droneplan.errors import InvalidInputError
from droneplan.geometry.obstacles import Obstacle

_POLYGONAL = ["Polygon", "MultiPolygon"]


def _read_wgs84(path: str | Path) -> gpd.GeoDataFrame:
    gdf = gpd.read_file(path)
    if gdf.crs is None:
        raise InvalidInputError(f"{path}: missing CRS (.prj)")
    return gdf.to_crs(epsg=4326)


def load_area(path: str | Path) -> BaseGeometry:
    gdf = _read_wgs84(path)
    polys = gdf.geometry[gdf.geometry.geom_type.isin(_POLYGONAL)]
    if polys.empty:
        raise InvalidInputError(f"{path}: no polygon features")
    valid = shapely.make_valid(shapely.union_all(polys.to_numpy()))
    area = shapely.union_all([g for g in shapely.get_parts(valid) if g.geom_type in _POLYGONAL])
    if area.is_empty or area.area <= 0:
        raise InvalidInputError(f"{path}: area is empty")
    return area


def load_points(path: str | Path) -> FloatArray:
    gdf = _read_wgs84(path)
    pts = gdf.geometry[gdf.geometry.geom_type == "Point"]
    if pts.empty:
        raise InvalidInputError(f"{path}: no point features")
    return np.column_stack([pts.x.to_numpy(), pts.y.to_numpy()]).astype(np.float64)


def load_obstacles(path: str | Path, height_field: str = "height_m") -> list[Obstacle]:
    gdf = _read_wgs84(path)
    polys = gdf[gdf.geometry.geom_type.isin(_POLYGONAL)]
    heights = (
        polys[height_field] if height_field in polys.columns else pd.Series(None, index=polys.index)
    )
    return [
        Obstacle(geom, None if pd.isna(h) else float(h))
        for geom, h in zip(polys.geometry, heights, strict=True)
    ]
