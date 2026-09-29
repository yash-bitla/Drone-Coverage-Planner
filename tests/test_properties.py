import numpy as np
from hypothesis import HealthCheck, assume, given, settings
from hypothesis import strategies as st
from hypothesis.extra.numpy import arrays
from shapely.geometry import box

from droneplan.config import DroneSpec, PlannerConfig, StationSpec
from droneplan.errors import InvalidInputError
from droneplan.geometry.grid import Grid, rasterize
from droneplan.geometry.routing import Router
from droneplan.planner import Algorithm, plan_on_grid

SETTINGS = settings(max_examples=60, deadline=None, suppress_health_check=[HealthCheck.too_slow])


@st.composite
def configs(draw: st.DrawFn) -> PlannerConfig:
    return PlannerConfig(
        drone=DroneSpec(
            count=draw(st.integers(1, 4)), range_m=draw(st.floats(400.0, 3000.0)), speed_mps=10.0
        ),
        station=StationSpec(
            charge_time_s=draw(st.floats(0.0, 600.0)), capacity=draw(st.integers(1, 2))
        ),
    )


def stations(draw: st.DrawFn, n: int) -> np.ndarray:
    coord = st.floats(-10.0, 130.0, allow_nan=False)
    return np.array([[draw(coord), draw(coord)] for _ in range(n)])


@SETTINGS
@given(
    arrays(np.bool_, st.tuples(st.integers(2, 10), st.integers(2, 10))),
    st.data(),
    st.sampled_from(list(Algorithm)),
)
def test_valid_plans_on_random_masks(
    mask: np.ndarray, data: st.DataObject, algorithm: Algorithm
) -> None:
    assume(mask.any())
    grid = Grid.from_mask(mask, 10.0, origin=(0.0, 10.0 * mask.shape[0]))
    plan = plan_on_grid(
        grid, stations(data.draw, data.draw(st.integers(1, 3))), data.draw(configs()), algorithm
    )
    assert plan.violations() == []
    assert plan.makespan_s >= plan.metrics().lower_bound_s - 1e-6


@SETTINGS
@given(st.data(), st.sampled_from(list(Algorithm)))
def test_valid_plans_around_obstacles(data: st.DataObject, algorithm: Algorithm) -> None:
    w, h = data.draw(st.integers(40, 120)), data.draw(st.integers(40, 120))
    size = st.floats(8.0, 30.0)
    obstacles = []
    for _ in range(data.draw(st.integers(1, 2))):
        x, y = data.draw(st.floats(0.0, w)), data.draw(st.floats(0.0, h))
        obstacles.append(box(x, y, x + data.draw(size), y + data.draw(size)))
    assume(len(obstacles) == 1 or obstacles[0].distance(obstacles[1]) > 25)
    try:
        grid = rasterize(box(0, 0, w, h), 10.0, max_cells=10_000, obstacles=obstacles)
    except InvalidInputError:
        assume(False)
    router = Router(obstacles)
    pts = stations(data.draw, data.draw(st.integers(1, 3)))
    assume(not router.contains(pts).any())
    cfg = data.draw(configs())
    near, _ = router.nearest(grid.centers(grid.required_cells()), pts)
    assume(2.0 * float(near.max()) <= cfg.drone.usable_range_m)
    plan = plan_on_grid(grid, pts, cfg, algorithm, router)
    assert plan.violations() == []
    assert plan.makespan_s >= plan.metrics().lower_bound_s - 1e-6
