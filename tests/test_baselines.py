import numpy as np
import pytest
from shapely.geometry import box

from droneplan.baselines.greedy_refuel import greedy_refuel
from droneplan.baselines.stc import stc_path
from droneplan.config import DroneSpec, PlannerConfig
from droneplan.geometry.grid import Grid, rasterize
from droneplan.geometry.routing import Router
from droneplan.kinematics import polyline_length
from droneplan.planner import Algorithm, plan_on_grid

CFG = PlannerConfig(drone=DroneSpec(count=3, range_m=5_000, speed_mps=12))


def test_stc_is_a_hamiltonian_circuit() -> None:
    full = np.ones((4, 4), dtype=bool)
    path = stc_path(full, (0, 0), allowed=full)
    assert len(path) == 16 == len({tuple(p) for p in path.tolist()})
    steps = np.abs(np.diff(np.vstack([path, path[:1]]), axis=0)).sum(axis=1)
    assert (steps == 1).all()


def test_stc_skips_blocked_and_odd_edge_mega_cells() -> None:
    region = np.ones((5, 5), dtype=bool)
    allowed = region.copy()
    allowed[0, 0] = False
    visited = {tuple(p) for p in stc_path(region, (4, 4), allowed).tolist()}
    assert (0, 0) not in visited and (0, 1) not in visited
    assert all(r < 4 and c < 4 for r, c in visited)


def test_greedy_refuel_flights_within_range() -> None:
    xy = np.column_stack([np.arange(1, 41) * 10.0, np.zeros(40)])
    rc = np.column_stack([np.zeros(40), np.arange(40)]).astype(np.int64)
    flights = greedy_refuel(xy, rc, np.array([[200.0, 0.0]]), 0, 400.0)
    assert len(flights) > 1
    assert all(polyline_length(f.path) <= 400.0 + 1e-9 for f in flights)
    assert {tuple(c) for f in flights for c in f.covered.tolist()} == {
        tuple(c) for c in rc.tolist()
    }


def test_greedy_refuel_resumes_at_next_point_after_a_long_step() -> None:
    xy = np.array([[450.0, 0.0], [-450.0, 0.0]])
    rc = np.array([[0, 0], [0, 1]], dtype=np.int64)
    flights = greedy_refuel(xy, rc, np.array([[0.0, 0.0]]), 0, 1000.0)
    assert len(flights) == 2
    assert all(polyline_length(f.path) <= 1000.0 + 1e-9 for f in flights)
    assert [tuple(c) for f in flights for c in f.covered.tolist()] == [(0, 0), (0, 1)]


@pytest.mark.parametrize("algorithm", [Algorithm.DARP_STC, Algorithm.DARP_BOUSTROPHEDON])
def test_baselines_valid_without_obstacles(algorithm: Algorithm) -> None:
    grid = Grid.from_mask(np.ones((6, 20), dtype=bool), 50.0, origin=(0.0, 300.0))
    assert plan_on_grid(grid, np.array([[-50.0, 150.0]]), CFG, algorithm).violations() == []


@pytest.mark.parametrize("algorithm", [Algorithm.DARP_STC, Algorithm.DARP_BOUSTROPHEDON])
def test_baselines_valid_with_obstacles(algorithm: Algorithm) -> None:
    wall = box(480, 60, 520, 400)
    grid = rasterize(box(0, 0, 1000, 300), 50.0, max_cells=1000, obstacles=[wall])
    plan = plan_on_grid(grid, np.array([[-50.0, 150.0]]), CFG, algorithm, Router([wall]))
    assert plan.violations() == []
