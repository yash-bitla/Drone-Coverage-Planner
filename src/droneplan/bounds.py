from __future__ import annotations

import numpy as np

from droneplan._types import FloatArray
from droneplan.config import PlannerConfig
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router


def lower_bound_s(
    grid: Grid, stations_xy: FloatArray, cfg: PlannerConfig, router: Router | None = None
) -> float:
    """max(workload, pad capacity, reach); each term alone bounds the makespan.

    workload: a flight visiting distinct cell centres c1..cm flies at least
    near(c1) + (m-1)s + near(cm) >= sum_i min(s, 2*near(ci)), so covering every required cell
    takes at least the sum of min(s, 2*near(c)) over them of flying; energy beyond the n full
    batteries must be recharged; n drones share flying and charging. pads: that recharging fits
    on the stations' pads. reach: the farthest cell needs an out-and-back from its nearest station.
    """
    d = cfg.drone
    near, _ = (router or Router()).nearest(grid.centers(grid.required_cells()), stations_xy)
    sweep_m = float(np.minimum(grid.cell_size, 2.0 * near).sum())
    charge_s = (
        max(0.0, sweep_m - d.count * d.usable_range_m) / d.range_m * cfg.station.charge_time_s
    )
    workload = (sweep_m / d.speed_mps + charge_s) / d.count
    pads = charge_s / (len(stations_xy) * cfg.station.capacity)
    return max(workload, pads, 2.0 * float(near.max()) / d.speed_mps)
