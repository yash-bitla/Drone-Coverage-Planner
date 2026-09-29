from __future__ import annotations

import heapq
from collections.abc import Sequence

import numpy as np

from droneplan._types import FloatArray
from droneplan.config import PlannerConfig
from droneplan.errors import InfeasiblePlanError
from droneplan.geometry.routing import Router
from droneplan.kinematics import polyline_length
from droneplan.scheduling.charging import ChargingNetwork
from droneplan.scheduling.flying import fly, landing_estimate
from droneplan.scheduling.model import Body, Flight, FlightKind, Schedule, initial_stations

_EPS = 1e-9
_NO_CELLS = np.zeros((0, 2), dtype=np.int64)


def list_schedule(
    bodies: Sequence[Body],
    stations_xy: FloatArray,
    cfg: PlannerConfig,
    router: Router | None = None,
) -> Schedule:
    """Longest sortie first, preferring the drone's current station; land where the charge
    finishes soonest, queue included. A drone that can reach nothing repositions."""
    router = router or Router()
    if not bodies:
        return Schedule((), cfg.drone.count)
    usable = cfg.drone.usable_range_m
    lengths = np.array([polyline_length(b.xy) for b in bodies])
    to_first = router.distances(np.array([b.xy[0] for b in bodies]), stations_xy)
    from_last = router.distances(np.array([b.xy[-1] for b in bodies]), stations_xy)
    hops = router.distances(stations_xy, stations_xy)
    home = to_first.argmin(axis=1)
    reachable = to_first + (lengths + from_last.min(axis=1))[:, None] <= usable + _EPS

    stranded = np.flatnonzero(~reachable[np.arange(len(bodies)), home])
    if len(stranded):
        raise InfeasiblePlanError(f"{len(stranded)} sorties exceed the range from every station")

    def path_via(s: int, b: Body, land: int) -> FloatArray:
        out = router.path(stations_xy[s], b.xy[0])[:-1]
        back = router.path(b.xy[-1], stations_xy[land])[1:]
        return np.vstack([out, b.xy, back])

    remaining = sorted(range(len(bodies)), key=lambda i: -lengths[i])
    network = ChargingNetwork(len(stations_xy), cfg.station.capacity)
    heap = [(0.0, d, s) for d, s in enumerate(initial_stations(cfg.drone.count, len(stations_xy)))]
    heapq.heapify(heap)
    flights: list[Flight] = []

    while remaining:
        if not heap:
            raise InfeasiblePlanError(f"{len(remaining)} sorties unreachable by any drone")
        t, drone, s = heapq.heappop(heap)
        candidates = [i for i in remaining if reachable[i, s]]
        choice = next(
            (i for i in candidates if home[i] == s), candidates[0] if candidates else None
        )

        if choice is None:
            target = int(home[remaining[0]])
            if hops[s, target] > usable + _EPS:
                continue
            path = router.path(stations_xy[s], stations_xy[target])
            f = fly(drone, FlightKind.REPOSITION, s, target, t, path, _NO_CELLS, network, cfg)
            flights.append(f)
            heapq.heappush(heap, (f.charge_end, drone, target))
            continue

        remaining.remove(choice)
        b = bodies[choice]
        best_end, best_land = np.inf, -1
        for land in range(len(stations_xy)):
            if to_first[choice, s] + lengths[choice] + from_last[choice, land] > usable + _EPS:
                continue
            _, start, duration, _ = landing_estimate(path_via(s, b, land), t, land, network, cfg)
            if start + duration < best_end:
                best_end, best_land = start + duration, land
        f = fly(
            drone,
            FlightKind.COVERAGE,
            s,
            best_land,
            t,
            path_via(s, b, best_land),
            b.rc,
            network,
            cfg,
        )
        flights.append(f)
        heapq.heappush(heap, (f.charge_end, drone, best_land))

    return Schedule(tuple(flights), cfg.drone.count)
