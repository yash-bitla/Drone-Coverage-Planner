from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from droneplan._types import FloatArray
from droneplan.bounds import lower_bound_s
from droneplan.config import PlannerConfig
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router
from droneplan.kinematics import count_turns
from droneplan.scheduling.model import Schedule


@dataclass(frozen=True, slots=True)
class PlanMetrics:
    algorithm: str
    makespan_s: float
    lower_bound_s: float
    gap_pct: float
    total_distance_m: float
    n_flights: int
    n_turns: int
    coverage_pct: float
    redundancy_pct: float
    unmappable_km2: float
    queue_wait_s: float
    utilization_pct: float
    solve_time_s: float

    def as_dict(self) -> dict[str, float | int | str]:
        return asdict(self)


def compute_metrics(
    schedule: Schedule,
    grid: Grid,
    stations_xy: FloatArray,
    cfg: PlannerConfig,
    router: Router,
    *,
    algorithm: str,
    solve_time_s: float,
) -> PlanMetrics:
    h, w = grid.shape
    visits = np.zeros(grid.shape, dtype=np.int64)
    for f in schedule.flights:
        rc = f.covered
        ok = (rc[:, 0] >= 0) & (rc[:, 0] < h) & (rc[:, 1] >= 0) & (rc[:, 1] < w)
        np.add.at(visits, (rc[ok, 0], rc[ok, 1]), 1)
    req = grid.required
    n_req = int(req.sum())
    makespan = schedule.makespan_s
    lb = lower_bound_s(grid, stations_xy, cfg, router)
    flying = sum(f.t_land - f.t_launch for f in schedule.flights)
    return PlanMetrics(
        algorithm=algorithm,
        makespan_s=makespan,
        lower_bound_s=lb,
        gap_pct=100.0 * (makespan / lb - 1.0) if lb > 0 else 0.0,
        total_distance_m=sum(f.distance_m for f in schedule.flights),
        n_flights=len(schedule.flights),
        n_turns=sum(count_turns(f.path) for f in schedule.flights),
        coverage_pct=100.0 * int(((visits > 0) & req).sum()) / n_req,
        redundancy_pct=100.0 * (int(visits[req].sum()) / n_req - 1.0),
        unmappable_km2=grid.n_unmappable * grid.cell_size**2 / 1e6,
        queue_wait_s=sum(f.charge_start - f.t_land for f in schedule.flights),
        utilization_pct=100.0 * flying / (schedule.n_drones * makespan) if makespan > 0 else 0.0,
        solve_time_s=solve_time_s,
    )
