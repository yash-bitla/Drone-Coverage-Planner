from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Sequence
from functools import partial

import numpy as np

from droneplan._types import FloatArray
from droneplan.config import PlannerConfig
from droneplan.coverage.tour import build_tour
from droneplan.errors import InfeasiblePlanError
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router
from droneplan.scheduling.list_scheduler import list_schedule
from droneplan.scheduling.model import Body, Schedule
from droneplan.sortie.split import Span, SplitInput, greedy_split, split_tour

_GOLDEN = (math.sqrt(5) - 1) / 2

Splitter = Callable[[SplitInput, float], list[Span]]


def solve_rss(
    grid: Grid,
    stations_xy: FloatArray,
    cfg: PlannerConfig,
    router: Router,
    *,
    splitters: Sequence[Splitter] = (split_tour, greedy_split),
    search_budget: bool = True,
) -> Schedule:
    """Best schedule over `splitters`, each with its own budget search.

    The DP split is optimal in energy, not makespan, so the greedy split sometimes wins.
    """
    tour = build_tour(grid, stations_xy, cfg.drone, router)
    inp = SplitInput.from_tour(tour.xy, stations_xy, router)
    is_cell = tour.rc[:, 0] >= 0
    usable = cfg.drone.usable_range_m
    lo = 2.0 * float(inp.near.max()) + grid.cell_size

    def evaluate(splitter: Splitter, budget_m: float) -> Schedule:
        bodies = [
            Body(
                tour.xy[s.start : s.end + 1],
                tour.rc[s.start : s.end + 1][is_cell[s.start : s.end + 1]],
            )
            for s in splitter(inp, budget_m)
        ]
        return list_schedule(bodies, stations_xy, cfg, router)

    def solve(splitter: Splitter) -> Schedule:
        if not search_budget or lo >= usable:
            return evaluate(splitter, usable)
        return search_budget_min_makespan(partial(evaluate, splitter), lo, usable)

    return min_feasible((partial(solve, s) for s in splitters), key=lambda s: s.makespan_s)


def min_feasible[T](attempts: Iterable[Callable[[], T]], key: Callable[[T], float]) -> T:
    """The best result among attempts that don't raise InfeasiblePlanError.

    Re-raises the last error if every attempt is infeasible.
    """
    results: list[T] = []
    error: InfeasiblePlanError | None = None
    for attempt in attempts:
        try:
            results.append(attempt())
        except InfeasiblePlanError as exc:
            error = exc
    if not results:
        assert error is not None
        raise error
    return min(results, key=key)


def search_budget_min_makespan(
    evaluate: Callable[[float], Schedule],
    lo: float,
    hi: float,
    *,
    grid_points: int = 12,
    refine_iters: int = 8,
) -> Schedule:
    """Log-spaced scan of the sortie budget, then golden-section refinement around the best.

    Makespan is not unimodal in the budget (sortie counts change in steps), hence the scan.
    `hi` is always evaluated, so the result is never worse than using the full range.
    """
    cache: dict[float, Schedule | None] = {}

    def makespan(budget: float) -> float:
        if budget not in cache:
            try:
                cache[budget] = evaluate(budget)
            except InfeasiblePlanError:
                cache[budget] = None
        s = cache[budget]
        return math.inf if s is None else s.makespan_s

    candidates = [float(b) for b in np.geomspace(lo, hi, grid_points)]
    best = int(np.argmin([makespan(b) for b in candidates]))
    a = math.log(candidates[max(best - 1, 0)])
    c = math.log(candidates[min(best + 1, len(candidates) - 1)])
    for _ in range(refine_iters):
        x1, x2 = c - _GOLDEN * (c - a), a + _GOLDEN * (c - a)
        if makespan(math.exp(x1)) <= makespan(math.exp(x2)):
            c = x2
        else:
            a = x1

    feasible = [s for s in cache.values() if s is not None]
    if not feasible:
        raise InfeasiblePlanError("no sortie budget yields a feasible schedule")
    return min(feasible, key=lambda s: s.makespan_s)
