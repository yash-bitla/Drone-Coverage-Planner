from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import box

from droneplan.bounds import lower_bound_s
from droneplan.config import DroneSpec, PlannerConfig, StationSpec
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router
from droneplan.metrics import compute_metrics
from droneplan.scheduling.list_scheduler import list_schedule
from droneplan.scheduling.model import Body, Schedule
from droneplan.validation import validate_schedule

CFG = PlannerConfig(
    drone=DroneSpec(count=2, range_m=1000, speed_mps=10, accel_mps2=2, reserve_frac=0),
    station=StationSpec(charge_time_s=100, capacity=1),
)
GRID = Grid.from_mask(np.ones((1, 2), dtype=bool), 100.0, origin=(0.0, 100.0))
STATIONS = np.array([[0.0, 50.0]])


def schedule() -> Schedule:
    bodies = [Body(GRID.centers(np.array([[0, c]])), np.array([[0, c]])) for c in (0, 1)]
    return list_schedule(bodies, STATIONS, CFG)


def kinds(s: Schedule, cfg: PlannerConfig = CFG, router: Router | None = None) -> set[str]:
    return {v.kind for v in validate_schedule(s, GRID, STATIONS, cfg, router)}


def test_reach_bound() -> None:
    grid = Grid.from_mask(np.ones((1, 1), dtype=bool), 100.0, origin=(0.0, 100.0))
    cfg = PlannerConfig(drone=DroneSpec(count=1, range_m=10_000, speed_mps=10))
    assert lower_bound_s(grid, np.array([[50.0, 350.0]]), cfg) == pytest.approx(60.0)


def test_valid_schedule_passes_and_respects_bound() -> None:
    s = schedule()
    assert kinds(s) == set()
    assert s.makespan_s >= lower_bound_s(GRID, STATIONS, CFG) - 1e-9


def test_validator_catches_violations() -> None:
    s = schedule()
    a, b = s.flights
    assert kinds(replace(s, flights=(a,))) == {"coverage"}
    clash = replace(b, t_land=a.t_land, charge_start=a.charge_start, charge_end=a.charge_end)
    assert "pad_capacity" in kinds(replace(s, flights=(a, clash)))
    assert "covered_off_path" in kinds(
        replace(s, flights=(replace(a, covered=np.array([[0, 1]])), b))
    )
    assert "range" in kinds(s, replace(CFG, drone=replace(CFG.drone, range_m=100)))
    assert "obstacle" in kinds(s, router=Router([box(90, 40, 110, 60)]))


def test_metrics() -> None:
    m = compute_metrics(
        schedule(), GRID, STATIONS, CFG, Router(), algorithm="rss", solve_time_s=0.1
    )
    assert (m.coverage_pct, m.redundancy_pct, m.n_flights, m.unmappable_km2) == (100.0, 0.0, 2, 0.0)
    assert m.as_dict()["algorithm"] == "rss"
