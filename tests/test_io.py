from pathlib import Path

import geopandas as gpd
import pytest
from shapely.geometry import Point, box

from droneplan.errors import InvalidInputError
from droneplan.io import load_area, load_obstacles, load_points


def test_round_trip(tmp_path: Path) -> None:
    gpd.GeoDataFrame(geometry=[box(78.0, 20.0, 78.01, 20.01)], crs=4326).to_file(tmp_path / "a.shp")
    gpd.GeoDataFrame(geometry=[Point(78.0, 20.0)], crs=4326).to_file(tmp_path / "s.geojson")
    gpd.GeoDataFrame(
        {"height_m": [45.0, None]},
        geometry=[box(78.002, 20.002, 78.003, 20.003), box(78.005, 20.005, 78.006, 20.006)],
        crs=4326,
    ).to_file(tmp_path / "o.geojson")

    assert load_area(tmp_path / "a.shp").bounds == pytest.approx((78.0, 20.0, 78.01, 20.01))
    assert load_points(tmp_path / "s.geojson").tolist() == [[78.0, 20.0]]
    assert [o.height_m for o in load_obstacles(tmp_path / "o.geojson")] == [45.0, None]


def test_missing_crs_rejected(tmp_path: Path) -> None:
    gpd.GeoDataFrame(geometry=[box(0, 0, 1, 1)]).to_file(tmp_path / "nocrs.shp")
    with pytest.raises(InvalidInputError):
        load_area(tmp_path / "nocrs.shp")
