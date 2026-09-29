import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st
from hypothesis.extra.numpy import arrays
from shapely.geometry import box

from droneplan.config import DroneSpec
from droneplan.coverage import _kernels
from droneplan.coverage.lanes import Lane, extract_lanes
from droneplan.coverage.tour import build_tour, lane_ends, transition_costs
from droneplan.geometry.grid import Grid, rasterize
from droneplan.geometry.routing import Router
from droneplan.kinematics import count_turns

DRONE = DroneSpec(count=1, range_m=10_000, speed_mps=10, accel_mps2=2)
WALL = box(95, -10, 105, 60)


def test_extract_lanes() -> None:
    mask = np.array([[1, 1, 0, 1], [0, 0, 0, 0], [1, 1, 1, 1]], dtype=bool)
    assert extract_lanes(mask) == [Lane(0, 0, 1), Lane(0, 3, 3), Lane(2, 0, 3)]


def test_rectangle_gives_boustrophedon() -> None:
    grid = Grid.from_mask(np.ones((4, 10), dtype=bool), 10.0, origin=(0, 40))
    tour = build_tour(grid, np.array([[-10.0, 35.0]]), DRONE, Router())
    assert len(tour) == tour.n_cells == 40
    assert count_turns(tour.xy) == 6
    assert tuple(tour.rc[0]) == (0, 0)


def test_tour_routes_around_a_wall() -> None:
    router = Router([WALL])
    grid = rasterize(box(0, 0, 200, 50), 10.0, max_cells=1000, obstacles=[WALL])
    tour = build_tour(grid, np.array([[-10.0, 25.0]]), DRONE, router)
    assert tour.n_cells == grid.n_required
    assert (tour.rc[:, 0] < 0).any()
    assert router.clear(tour.xy[:-1], tour.xy[1:]).all()


def test_transition_costs_are_symmetric() -> None:
    grid = rasterize(box(0, 0, 200, 50), 10.0, max_cells=1000, obstacles=[WALL])
    cost = transition_costs(lane_ends(grid, extract_lanes(grid.required)), Router([WALL]), DRONE)
    np.testing.assert_allclose(cost, cost.T)


def test_two_opt_never_worsens() -> None:
    rng = np.random.default_rng(7)
    cost = rng.uniform(1, 100, (60, 60))
    cost = (cost + cost.T) / 2
    order = rng.permutation(30).astype(np.int64)
    orient = rng.integers(0, 2, 30).astype(np.int64)
    before = _kernels.path_cost(order, orient, cost)
    _kernels.two_opt(order, orient, cost, 50)
    assert _kernels.path_cost(order, orient, cost) <= before + 1e-9
    assert sorted(order.tolist()) == list(range(30))


@settings(max_examples=50, deadline=None)
@given(arrays(np.bool_, st.tuples(st.integers(1, 12), st.integers(1, 12))))
def test_tour_visits_every_required_cell_once(mask: np.ndarray) -> None:
    if not mask.any():
        return
    grid = Grid.from_mask(mask, 10.0, origin=(0, 10.0 * mask.shape[0]))
    tour = build_tour(grid, np.array([[0.0, 0.0]]), DRONE, Router())
    cells = [tuple(c) for c in tour.rc.tolist()]
    assert len(set(cells)) == len(cells) == int(mask.sum())
    assert all(mask[r, c] for r, c in cells)
