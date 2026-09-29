import json
from pathlib import Path

import geopandas as gpd
import numpy as np
import pytest
from shapely.geometry import Point, box

from droneplan.cli import main
from droneplan.config import DroneSpec, PlannerConfig
from droneplan.export import plan_to_geojson
from droneplan.geometry.obstacles import Obstacle
from droneplan.planner import plan_area

AREA = box(78.0, 20.0, 78.01, 20.008)
TOWER = box(78.004, 20.003, 78.005, 20.004)
CFG = PlannerConfig(drone=DroneSpec(count=2, range_m=8_000, speed_mps=12))


def test_geojson_has_timed_flights_and_obstacles() -> None:
    gj = plan_to_geojson(
        plan_area(AREA, np.array([[78.0, 20.0]]), CFG, obstacles=[Obstacle(TOWER)])
    )
    kinds = [f["properties"]["kind"] for f in gj["features"]]
    assert kinds.count("station") == 1 and kinds.count("obstacle") == 1
    for f in (f for f in gj["features"] if f["properties"]["kind"] == "flight"):
        ts = f["properties"]["timestamps"]
        assert len(ts) == len(f["geometry"]["coordinates"]) and ts == sorted(ts)
    assert gj["properties"]["metrics"]["coverage_pct"] == 100.0


def test_cli_writes_plan(tmp_path: Path) -> None:
    gpd.GeoDataFrame(geometry=[AREA], crs=4326).to_file(tmp_path / "area.geojson")
    gpd.GeoDataFrame(geometry=[Point(78.0, 20.0)], crs=4326).to_file(tmp_path / "st.geojson")
    gpd.GeoDataFrame({"height_m": [150.0]}, geometry=[TOWER], crs=4326).to_file(
        tmp_path / "obs.geojson"
    )
    out = tmp_path / "plan.geojson"
    code = main(
        [
            str(tmp_path / "area.geojson"),
            "--stations",
            str(tmp_path / "st.geojson"),
            "--obstacles",
            str(tmp_path / "obs.geojson"),
            "--drones",
            "2",
            "--range-km",
            "8",
            "--speed",
            "12",
            "--out",
            str(out),
        ]
    )
    assert code == 0
    assert json.loads(out.read_text())["type"] == "FeatureCollection"


def test_cli_missing_area_reports_error(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    gpd.GeoDataFrame(geometry=[Point(78.0, 20.0)], crs=4326).to_file(tmp_path / "st.geojson")
    code = main(
        [
            str(tmp_path / "missing.geojson"),
            "--stations",
            str(tmp_path / "st.geojson"),
            "--drones",
            "2",
            "--range-km",
            "8",
            "--speed",
            "12",
            "--out",
            str(tmp_path / "plan.geojson"),
        ]
    )
    assert code == 2
    assert capsys.readouterr().err.startswith("error:")


def test_cli_unwritable_out_reports_error(tmp_path: Path) -> None:
    gpd.GeoDataFrame(geometry=[AREA], crs=4326).to_file(tmp_path / "area.geojson")
    gpd.GeoDataFrame(geometry=[Point(78.0, 20.0)], crs=4326).to_file(tmp_path / "st.geojson")
    code = main(
        [
            str(tmp_path / "area.geojson"),
            "--stations",
            str(tmp_path / "st.geojson"),
            "--drones",
            "2",
            "--range-km",
            "8",
            "--speed",
            "12",
            "--out",
            str(tmp_path / "no_such_dir" / "plan.geojson"),
        ]
    )
    assert code == 2


def test_geojson_carries_config_for_the_viewer() -> None:
    gj = plan_to_geojson(plan_area(AREA, np.array([[78.0, 20.0]]), CFG))
    assert gj["properties"]["config"] == {
        "drones": 2,
        "range_m": 8000.0,
        "usable_range_m": 7200.0,
        "speed_mps": 12.0,
        "charge_time_s": 1800.0,
        "capacity": 1,
    }
