import itertools

import numpy as np
import pytest
from shapely.geometry import box

from droneplan.errors import InfeasiblePlanError
from droneplan.geometry.routing import Router
from droneplan.sortie.split import Span, SplitInput, greedy_split, split_tour

LINE = np.array([[10.0, 0.0], [20.0, 0.0], [30.0, 0.0], [40.0, 0.0]])


def test_single_sortie_when_budget_allows() -> None:
    assert split_tour(SplitInput.from_tour(LINE, np.array([[0.0, 0.0]])), 80.0) == [Span(0, 3)]


def test_two_stations_prefer_two_short_sorties() -> None:
    inp = SplitInput.from_tour(LINE, np.array([[0.0, 0.0], [50.0, 0.0]]))
    assert split_tour(inp, 45.0) == [Span(0, 1), Span(2, 3)]


def test_unreachable_point_raises() -> None:
    with pytest.raises(InfeasiblePlanError):
        split_tour(SplitInput.from_tour(LINE, np.array([[0.0, 0.0]])), 79.0)


def test_obstacle_lengthens_the_trip_home() -> None:
    point, station = np.array([[100.0, 0.0]]), np.array([[0.0, 0.0]])
    assert split_tour(SplitInput.from_tour(point, station), 204.0) == [Span(0, 0)]
    with pytest.raises(InfeasiblePlanError):
        split_tour(SplitInput.from_tour(point, station, Router([box(40, -10, 60, 10)])), 204.0)


def _brute_force(inp: SplitInput, m: int, budget: float) -> float:
    best = np.inf
    for cuts in itertools.product([False, True], repeat=m - 1):
        bounds = [0, *[i + 1 for i, c in enumerate(cuts) if c], m]
        costs = [
            inp.sortie_distance(Span(bounds[t], bounds[t + 1] - 1)) for t in range(len(bounds) - 1)
        ]
        if all(c <= budget for c in costs):
            best = min(best, sum(costs))
    return best


@pytest.mark.parametrize("seed", range(25))
def test_dp_matches_brute_force(seed: int) -> None:
    rng = np.random.default_rng(seed)
    xy = rng.uniform(0, 100, (int(rng.integers(2, 10)), 2))
    inp = SplitInput.from_tour(xy, rng.uniform(0, 100, (int(rng.integers(1, 3)), 2)))
    budget = float(rng.uniform(250, 500))
    expected = _brute_force(inp, len(xy), budget)
    if not np.isfinite(expected):
        with pytest.raises(InfeasiblePlanError):
            split_tour(inp, budget)
        return
    assert sum(inp.sortie_distance(s) for s in split_tour(inp, budget)) == pytest.approx(expected)


def test_greedy_is_feasible_and_never_better() -> None:
    rng = np.random.default_rng(3)
    xy = np.cumsum(rng.uniform(5, 15, (60, 2)), axis=0)
    inp = SplitInput.from_tour(xy, np.array([[0.0, 0.0], xy[-1]]))
    budget = 2.5 * float(np.hypot(*xy[30]))
    greedy, optimal = greedy_split(inp, budget), split_tour(inp, budget)
    assert all(inp.sortie_distance(s) <= budget + 1e-9 for s in greedy)
    assert sum(map(inp.sortie_distance, optimal)) <= sum(map(inp.sortie_distance, greedy)) + 1e-9
    assert (greedy[0].start, greedy[-1].end) == (0, len(xy) - 1)
