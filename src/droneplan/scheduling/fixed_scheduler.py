from __future__ import annotations

import heapq
from collections.abc import Sequence
from dataclasses import dataclass

from droneplan._types import FloatArray, IntArray
from droneplan.config import PlannerConfig
from droneplan.scheduling.charging import ChargingNetwork
from droneplan.scheduling.flying import fly
from droneplan.scheduling.model import Flight, FlightKind, Schedule


@dataclass(frozen=True, eq=False)
class PlannedFlight:
    path: FloatArray
    covered: IntArray
    launch_station: int
    land_station: int


def fixed_schedule(
    per_drone: Sequence[Sequence[PlannedFlight]], n_stations: int, cfg: PlannerConfig
) -> Schedule:
    network = ChargingNetwork(n_stations, cfg.station.capacity)
    heap = [(0.0, d, 0) for d, planned in enumerate(per_drone) if planned]
    heapq.heapify(heap)
    flights: list[Flight] = []
    while heap:
        t, drone, idx = heapq.heappop(heap)
        pf = per_drone[drone][idx]
        f = fly(
            drone,
            FlightKind.COVERAGE,
            pf.launch_station,
            pf.land_station,
            t,
            pf.path,
            pf.covered,
            network,
            cfg,
        )
        flights.append(f)
        if idx + 1 < len(per_drone[drone]):
            heapq.heappush(heap, (f.charge_end, drone, idx + 1))
    return Schedule(tuple(flights), len(per_drone))
