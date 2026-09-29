import math

import numpy as np
import pytest
from shapely import affinity
from shapely.geometry import Polygon, box

from droneplan.config import DroneSpec, PlannerConfig
from droneplan.errors import InvalidInputError
from droneplan.geometry.frame import Frame, utm_crs_for
from droneplan.geometry.grid import Grid
from droneplan.geometry.obstacles import Obstacle
from droneplan.geometry.routing import Router
from droneplan.planner import Algorithm, plan_area, plan_on_grid

CFG = PlannerConfig(drone=DroneSpec(count=3, range_m=5_000, speed_mps=12))
GRID = Grid.from_mask(np.ones((6, 20), dtype=bool), 50.0, origin=(0.0, 300.0))
STATIONS = np.array([[-50.0, 150.0]])
BASE = Frame(utm_crs_for(78.0, 20.0))
CX, CY = BASE.lnglat_to_utm(np.array([[78.0, 20.0]]))[0]


def to_lnglat(geom_utm: Polygon) -> Polygon:
    return Polygon(BASE.utm_to_lnglat(np.asarray(geom_utm.exterior.coords)))


RECT = to_lnglat(
    affinity.rotate(box(CX - 1000, CY - 300, CX + 1000, CY + 300), 25, origin=(CX, CY))
)
TOWER = to_lnglat(box(CX - 100, CY - 100, CX + 100, CY + 100))
STATION = BASE.utm_to_lnglat(
    np.array([[CX - 900 * math.cos(math.radians(25)), CY - 900 * math.sin(math.radians(25))]])
)


@pytest.mark.parametrize(
    "algorithm", [Algorithm.RSS, Algorithm.RSS_GREEDY_SPLIT, Algorithm.RSS_FULL_BUDGET]
)
def test_rss_variants_are_valid(algorithm: Algorithm) -> None:
    plan = plan_on_grid(GRID, STATIONS, CFG, algorithm)
    assert plan.violations() == []
    assert plan.makespan_s >= plan.metrics().lower_bound_s - 1e-9


def test_budget_search_never_worse_than_full_budget() -> None:
    searched = plan_on_grid(GRID, STATIONS, CFG, Algorithm.RSS)
    full = plan_on_grid(GRID, STATIONS, CFG, Algorithm.RSS_FULL_BUDGET)
    assert searched.makespan_s <= full.makespan_s + 1e-9


def test_plan_area_rotates_to_long_edge() -> None:
    plan = plan_area(RECT, STATION, CFG)
    assert plan.frame is not None
    assert math.degrees(plan.frame.angle_rad) % 180 == pytest.approx(25.0, abs=0.5)
    assert plan.violations() == []


def test_blocking_obstacle_is_avoided_and_reported() -> None:
    plan = plan_area(RECT, STATION, CFG, obstacles=[Obstacle(TOWER)])
    assert plan.violations() == []
    assert plan.metrics().unmappable_km2 > 0


def test_low_obstacle_is_flown_over() -> None:
    plan = plan_area(RECT, STATION, CFG, obstacles=[Obstacle(TOWER, height_m=50)])
    assert plan.metrics().unmappable_km2 == 0


def test_rejects_bad_stations() -> None:
    with pytest.raises(InvalidInputError):
        plan_on_grid(GRID, np.empty((0, 2)), CFG)
    with pytest.raises(InvalidInputError):
        plan_on_grid(GRID, STATIONS, CFG, router=Router([box(-60, 140, -40, 160)]))
