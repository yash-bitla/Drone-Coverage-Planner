from __future__ import annotations

import numpy as np

from droneplan._types import FloatArray, IntArray
from droneplan.errors import InfeasiblePlanError
from droneplan.geometry.routing import Router
from droneplan.scheduling.fixed_scheduler import PlannedFlight

_EPS = 1e-9


def greedy_refuel(
    path_xy: FloatArray,
    path_rc: IntArray,
    stations_xy: FloatArray,
    start_station: int,
    usable_range_m: float,
    router: Router | None = None,
) -> list[PlannedFlight]:
    """v1's refuelling rule: follow the path and break off to the nearest station only when
    the next point would leave too little range to get back."""
    router = router or Router()
    if len(path_xy) == 0:
        return []
    near, near_idx = router.nearest(path_xy, stations_xy)
    if (2 * near > usable_range_m + _EPS).any():
        raise InfeasiblePlanError("some cells are out of round-trip range of every station")
    steps = np.hypot(*np.diff(path_xy, axis=0).T) if len(path_xy) > 1 else np.empty(0)

    def planned(i0: int, k: int, launch: int, land: int) -> PlannedFlight:
        path = np.vstack(
            [
                router.path(stations_xy[launch], path_xy[i0])[:-1],
                path_xy[i0 : k + 1],
                router.path(path_xy[k], stations_xy[land])[1:],
            ]
        )
        covered = path_rc[i0 : k + 1]
        return PlannedFlight(path, covered[covered[:, 0] >= 0], launch, land)

    launch, i0, k, last = start_station, 0, 0, len(path_xy) - 1
    used = float(router.distances(stations_xy[launch], path_xy[0])[0, 0])
    if used + near[0] > usable_range_m + _EPS:
        raise InfeasiblePlanError("first cell is out of range of the drone's start station")
    flights: list[PlannedFlight] = []
    while k < last:
        if used + steps[k] + near[k + 1] <= usable_range_m + _EPS:
            used += float(steps[k])
            k += 1
            continue
        land = int(near_idx[k])
        flights.append(planned(i0, k, launch, land))
        launch, i0, used = land, k, float(near[k])
        if used + steps[k] + near[k + 1] > usable_range_m + _EPS:
            raise InfeasiblePlanError("range too short to advance along the path")
    flights.append(planned(i0, last, launch, int(near_idx[last])))
    return flights
