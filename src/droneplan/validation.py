from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from droneplan._types import FloatArray
from droneplan.config import PlannerConfig
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router
from droneplan.kinematics import polyline_time
from droneplan.scheduling.model import Flight, Schedule, charge_duration_s, initial_stations

_EPS = 1e-6


@dataclass(frozen=True, slots=True)
class Violation:
    kind: str
    detail: str


def _keys(xy: FloatArray) -> set[tuple[float, float]]:
    return {(round(float(x), 3), round(float(y), 3)) for x, y in xy}


def _flight_violations(
    i: int, f: Flight, grid: Grid, stations_xy: FloatArray, cfg: PlannerConfig
) -> list[Violation]:
    out = []
    if len(f.covered) and not _keys(grid.centers(f.covered)) <= _keys(f.path):
        out.append(Violation("covered_off_path", f"flight {i} claims cells it never passes over"))
    if f.distance_m > cfg.drone.usable_range_m + _EPS:
        out.append(Violation("range", f"flight {i} is {f.distance_m:.1f} m"))
    if not (
        np.allclose(f.path[0], stations_xy[f.launch_station], rtol=0.0, atol=_EPS)
        and np.allclose(f.path[-1], stations_xy[f.land_station], rtol=0.0, atol=_EPS)
    ):
        out.append(Violation("endpoints", f"flight {i} does not start and end at its stations"))
    if (
        f.t_land - f.t_launch
        < polyline_time(f.path, cfg.drone.speed_mps, cfg.drone.accel_mps2) - _EPS
    ):
        out.append(Violation("timing", f"flight {i} lands sooner than the kinematics allow"))
    if (
        f.charge_start < f.t_land - _EPS
        or f.charge_end - f.charge_start < charge_duration_s(f.distance_m, cfg) - _EPS
    ):
        out.append(Violation("charging", f"flight {i} charges too early or too briefly"))
    return out


def validate_schedule(
    schedule: Schedule,
    grid: Grid,
    stations_xy: FloatArray,
    cfg: PlannerConfig,
    router: Router | None = None,
) -> list[Violation]:
    router = router or Router()
    out: list[Violation] = []
    covered = np.zeros(grid.shape, dtype=bool)
    for i, f in enumerate(schedule.flights):
        out += _flight_violations(i, f, grid, stations_xy, cfg)
        rc = grid.inside(f.covered)
        covered[rc[:, 0], rc[:, 1]] = True

    missing = int((grid.required & ~covered).sum())
    if missing:
        out.append(Violation("coverage", f"{missing} mappable cells never covered"))

    if router.has_obstacles and schedule.flights:
        starts = np.vstack([f.path[:-1] for f in schedule.flights])
        ends = np.vstack([f.path[1:] for f in schedule.flights])
        crossing = int((~router.clear(starts, ends)).sum())
        if crossing:
            out.append(Violation("obstacle", f"{crossing} path segments enter an obstacle"))

    first_station = initial_stations(schedule.n_drones, len(stations_xy))
    for drone, flights in schedule.by_drone().items():
        if not flights:
            continue
        for prev, f in zip([None, *flights[:-1]], flights, strict=True):
            if prev is None and f.launch_station != first_station[drone]:
                out.append(Violation("sequence", f"drone {drone} starts at the wrong station"))
            elif prev is not None and (
                f.t_launch < prev.charge_end - _EPS or f.launch_station != prev.land_station
            ):
                out.append(Violation("sequence", f"drone {drone} launches uncharged or elsewhere"))

    for station in range(len(stations_xy)):
        events = sorted(
            (t, delta)
            for f in schedule.flights
            if f.land_station == station and f.charge_end - f.charge_start > _EPS
            for t, delta in ((f.charge_start, 1), (f.charge_end - _EPS, -1))
        )
        active = 0
        for _, delta in events:
            active += delta
            if active > cfg.station.capacity:
                out.append(Violation("pad_capacity", f"station {station} over capacity"))
                break
    return out
