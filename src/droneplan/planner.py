from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from enum import StrEnum
from functools import partial

import numpy as np
import numpy.typing as npt
from shapely.geometry.base import BaseGeometry

from droneplan._types import FloatArray
from droneplan.config import PlannerConfig
from droneplan.errors import InvalidInputError
from droneplan.geometry.frame import Frame
from droneplan.geometry.grid import Grid, rasterize
from droneplan.geometry.obstacles import Obstacle, inflate
from droneplan.geometry.routing import Router
from droneplan.geometry.sweep import min_width_sweep_angle
from droneplan.metrics import PlanMetrics, compute_metrics
from droneplan.scheduling.model import Schedule
from droneplan.solvers.area_first import solve_darp_boustrophedon, solve_darp_stc
from droneplan.solvers.rss import solve_rss
from droneplan.validation import Violation, validate_schedule


class Algorithm(StrEnum):
    RSS = "rss"
    RSS_FIXED_ANGLE = "rss-fixed-angle"
    RSS_GREEDY_SPLIT = "rss-greedy-split"
    RSS_FULL_BUDGET = "rss-full-budget"
    DARP_STC = "darp-stc"
    DARP_BOUSTROPHEDON = "darp-boustrophedon"

    @property
    def optimizes_sweep_angle(self) -> bool:
        return self in (Algorithm.RSS, Algorithm.RSS_GREEDY_SPLIT, Algorithm.RSS_FULL_BUDGET)


Solver = Callable[[Grid, FloatArray, PlannerConfig, Router], Schedule]

SOLVERS: dict[Algorithm, Solver] = {
    Algorithm.RSS: solve_rss,
    Algorithm.RSS_FIXED_ANGLE: solve_rss,  # same solver; plan_area skips the rotation
    Algorithm.RSS_GREEDY_SPLIT: partial(solve_rss, greedy=True),
    Algorithm.RSS_FULL_BUDGET: partial(solve_rss, search_budget=False),
    Algorithm.DARP_STC: solve_darp_stc,
    Algorithm.DARP_BOUSTROPHEDON: solve_darp_boustrophedon,
}


@dataclass(frozen=True, eq=False)
class Plan:
    algorithm: Algorithm
    grid: Grid
    stations_xy: FloatArray
    config: PlannerConfig
    schedule: Schedule
    solve_time_s: float
    router: Router
    frame: Frame | None = None

    @property
    def makespan_s(self) -> float:
        return self.schedule.makespan_s

    def metrics(self) -> PlanMetrics:
        return compute_metrics(
            self.schedule,
            self.grid,
            self.stations_xy,
            self.config,
            self.router,
            algorithm=self.algorithm.value,
            solve_time_s=self.solve_time_s,
        )

    def violations(self) -> list[Violation]:
        return validate_schedule(
            self.schedule, self.grid, self.stations_xy, self.config, self.router
        )


def plan_on_grid(
    grid: Grid,
    stations_xy: npt.ArrayLike,
    cfg: PlannerConfig,
    algorithm: Algorithm = Algorithm.RSS,
    router: Router | None = None,
) -> Plan:
    router = router or Router()
    stations = np.asarray(stations_xy, dtype=np.float64).reshape(-1, 2)
    if len(stations) == 0:
        raise InvalidInputError("at least one charging station is required")
    if router.contains(stations).any():
        raise InvalidInputError("a charging station lies inside an obstacle's clearance zone")
    if algorithm not in SOLVERS:
        raise InvalidInputError(f"unknown algorithm {algorithm!r}")
    t0 = time.perf_counter()
    schedule = SOLVERS[algorithm](grid, stations, cfg, router)
    return Plan(algorithm, grid, stations, cfg, schedule, time.perf_counter() - t0, router)


def plan_area(
    area_lnglat: BaseGeometry,
    stations_lnglat: npt.ArrayLike,
    cfg: PlannerConfig,
    algorithm: Algorithm = Algorithm.RSS,
    obstacles: Sequence[Obstacle] = (),
) -> Plan:
    base = Frame.for_geometry(area_lnglat)
    area_utm = base.geometry_to_utm(area_lnglat)
    angle = min_width_sweep_angle(area_utm) if algorithm.optimizes_sweep_angle else 0.0
    pivot = area_utm.centroid
    frame = base.rotated(angle, (pivot.x, pivot.y))
    footprints = [
        frame.geometry_to_plan(o.geometry)
        for o in obstacles
        if o.blocks(cfg.sensor.altitude_m, cfg.clearance_m)
    ]
    inflated = inflate(
        footprints, clearance_m=cfg.clearance_m, simplify_m=cfg.sensor.footprint_m / 4
    )
    grid = rasterize(
        frame.geometry_to_plan(area_lnglat),
        cfg.sensor.footprint_m,
        max_cells=cfg.max_cells,
        obstacles=inflated,
    )
    stations = frame.to_plan(np.asarray(stations_lnglat, dtype=np.float64).reshape(-1, 2))
    plan = plan_on_grid(grid, stations, cfg, algorithm, Router(inflated))
    return replace(plan, frame=frame)
