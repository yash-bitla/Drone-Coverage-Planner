"""Area-first baselines: DARP divides the area, each drone covers its region, then v1's
greedy refuelling cuts the path into flights."""

from __future__ import annotations

from typing import Literal

import numpy as np

from droneplan._types import BoolArray, FloatArray, IntArray
from droneplan.baselines.darp import darp, farthest_point_seeds
from droneplan.baselines.greedy_refuel import greedy_refuel
from droneplan.baselines.stc import stc_path
from droneplan.config import PlannerConfig
from droneplan.coverage.tour import build_tour
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router
from droneplan.scheduling.fixed_scheduler import PlannedFlight, fixed_schedule
from droneplan.scheduling.model import Schedule, initial_stations


def _stc_with_leftovers(
    grid: Grid,
    region: BoolArray,
    station_xy: FloatArray,
    stations_xy: FloatArray,
    cfg: PlannerConfig,
    router: Router,
) -> tuple[FloatArray, IntArray]:
    region_cells = np.argwhere(region)
    start = region_cells[int(np.argmin(np.hypot(*(grid.centers(region_cells) - station_xy).T)))]
    start_rc = (int(start[0]), int(start[1]))
    rc = stc_path(region, start_rc, allowed=~grid.blocked)
    leftover = region.copy()
    leftover[rc[:, 0], rc[:, 1]] = False
    if leftover.any():
        extra = build_tour(grid, stations_xy, cfg.drone, router, mask=leftover).rc
        rc = np.vstack([rc, extra[extra[:, 0] >= 0]])
    return router.connect(grid.centers(rc), rc)


def _area_first(
    grid: Grid,
    stations_xy: FloatArray,
    cfg: PlannerConfig,
    router: Router,
    coverage: Literal["stc", "lanes"],
) -> Schedule:
    n = cfg.drone.count
    starts = initial_stations(n, len(stations_xy))
    cells = grid.required_cells()
    seeds = cells[farthest_point_seeds(grid.centers(cells), stations_xy[starts[0]], n)]
    assignment = darp(grid.required, seeds)
    per_drone: list[list[PlannedFlight]] = []
    for d in range(n):
        region = assignment == d
        if not region.any():
            per_drone.append([])
            continue
        if coverage == "stc":
            xy, rc = _stc_with_leftovers(
                grid, region, stations_xy[starts[d]], stations_xy, cfg, router
            )
        else:
            tour = build_tour(grid, stations_xy, cfg.drone, router, mask=region)
            xy, rc = tour.xy, tour.rc
        per_drone.append(
            greedy_refuel(xy, rc, stations_xy, starts[d], cfg.drone.usable_range_m, router)
        )
    return fixed_schedule(per_drone, len(stations_xy), cfg)


def solve_darp_stc(
    grid: Grid, stations_xy: FloatArray, cfg: PlannerConfig, router: Router
) -> Schedule:
    return _area_first(grid, stations_xy, cfg, router, "stc")


def solve_darp_boustrophedon(
    grid: Grid, stations_xy: FloatArray, cfg: PlannerConfig, router: Router
) -> Schedule:
    return _area_first(grid, stations_xy, cfg, router, "lanes")
