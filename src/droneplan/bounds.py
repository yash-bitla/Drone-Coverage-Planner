from __future__ import annotations

from droneplan._types import FloatArray
from droneplan.config import PlannerConfig
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router


def lower_bound_s(
    grid: Grid, stations_xy: FloatArray, cfg: PlannerConfig, router: Router | None = None
) -> float:
    """max(workload, pad capacity, reach); each term alone bounds the makespan.

    workload: a flight covering m distinct cell centres flies at least (m-1)s plus two station
    legs of at least delta each, so the n_required cells need at least n_required * min(s, 2*delta)
    of flying; energy beyond the n full batteries must be recharged; n drones share flying and
    charging. pads: that recharging fits on the stations' pads. reach: the farthest cell needs an
    out-and-back from its nearest station.
    """
    d = cfg.drone
    near, _ = (router or Router()).nearest(grid.centers(grid.required_cells()), stations_xy)
    delta = float(near.min())
    sweep_m = grid.n_required * min(grid.cell_size, 2.0 * delta)
    charge_s = (
        max(0.0, sweep_m - d.count * d.usable_range_m) / d.range_m * cfg.station.charge_time_s
    )
    workload = (sweep_m / d.speed_mps + charge_s) / d.count
    pads = charge_s / (len(stations_xy) * cfg.station.capacity)
    return max(workload, pads, 2.0 * float(near.max()) / d.speed_mps)
