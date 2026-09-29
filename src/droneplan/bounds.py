from __future__ import annotations

from droneplan._types import FloatArray
from droneplan.config import PlannerConfig
from droneplan.coverage.lanes import extract_lanes
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router


def lower_bound_s(
    grid: Grid, stations_xy: FloatArray, cfg: PlannerConfig, router: Router | None = None
) -> float:
    """max(workload, pad capacity, reach); each term alone bounds the makespan.

    workload: every lane's centre line must be flown, and energy beyond the n full batteries
    must be recharged, all shared by n drones. pads: that recharging fits on the stations'
    pads. reach: the farthest cell needs an out-and-back from its nearest station.
    """
    d = cfg.drone
    sweep_m = sum(lane.n_cells - 1 for lane in extract_lanes(grid.required)) * grid.cell_size
    charge_s = (
        max(0.0, sweep_m - d.count * d.usable_range_m) / d.range_m * cfg.station.charge_time_s
    )
    workload = (sweep_m / d.speed_mps + charge_s) / d.count
    pads = charge_s / (len(stations_xy) * cfg.station.capacity)
    near, _ = (router or Router()).nearest(grid.centers(grid.required_cells()), stations_xy)
    return max(workload, pads, 2.0 * float(near.max()) / d.speed_mps)
