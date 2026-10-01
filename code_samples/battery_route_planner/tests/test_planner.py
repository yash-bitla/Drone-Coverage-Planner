import itertools

import numpy as np
import pytest
from shapely.geometry import box

from battery_route_planner import InfeasibleRouteError, Router, plan_sorties
from battery_route_planner.planner import Span, SplitInput, split_route

LINE = np.array([[10.0, 0.0], [20.0, 0.0], [30.0, 0.0], [40.0, 0.0]])


def test_two_stations_split_route_into_two_short_sorties() -> None:
    stations = np.array([[0.0, 0.0], [50.0, 0.0]])
    inp = SplitInput.from_route(LINE, stations, Router())

    assert split_route(inp, 45.0) == [Span(0, 1), Span(2, 3)]


def test_obstacle_can_make_previously_feasible_point_unreachable() -> None:
    point = np.array([[100.0, 0.0]])
    station = np.array([[0.0, 0.0]])

    assert plan_sorties(point, station, 204.0).sortie_count == 1

    with pytest.raises(InfeasibleRouteError):
        plan_sorties(
            point,
            station,
            204.0,
            obstacles=[box(40.0, -10.0, 60.0, 10.0)],
        )


def test_dp_matches_brute_force_optimum() -> None:
    rng = np.random.default_rng(7)

    for _ in range(20):
        x = np.cumsum(rng.uniform(5.0, 15.0, size=int(rng.integers(2, 9))))
        route = np.column_stack([x, np.zeros_like(x)])
        stations = np.array([[0.0, 0.0], [float(x[-1] + 10.0), 0.0]])
        budget = float(rng.uniform(35.0, 90.0))

        inp = SplitInput.from_route(route, stations, Router())
        expected = _brute_force(inp, budget)

        if not np.isfinite(expected):
            with pytest.raises(InfeasibleRouteError):
                split_route(inp, budget)
            continue

        spans = split_route(inp, budget)
        actual = sum(inp.sortie_distance(span) for span in spans)
        assert actual == pytest.approx(expected)


def _brute_force(inp: SplitInput, budget: float) -> float:
    point_count = len(inp.near)
    best = np.inf

    for cuts in itertools.product([False, True], repeat=point_count - 1):
        bounds = [0, *[i + 1 for i, cut in enumerate(cuts) if cut], point_count]
        spans = [Span(bounds[i], bounds[i + 1] - 1) for i in range(len(bounds) - 1)]
        costs = [inp.sortie_distance(span) for span in spans]

        if all(cost <= budget + 1e-9 for cost in costs):
            best = min(best, sum(costs))

    return best
