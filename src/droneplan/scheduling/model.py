from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from droneplan._types import FloatArray, IntArray
from droneplan.config import PlannerConfig
from droneplan.kinematics import polyline_length


class FlightKind(StrEnum):
    COVERAGE = "coverage"
    REPOSITION = "reposition"


@dataclass(frozen=True, eq=False)
class Flight:
    drone: int
    kind: FlightKind
    launch_station: int
    land_station: int
    t_launch: float
    t_land: float
    charge_start: float
    charge_end: float
    path: FloatArray
    covered: IntArray

    @property
    def distance_m(self) -> float:
        return polyline_length(self.path)


@dataclass(frozen=True, eq=False)
class Schedule:
    flights: tuple[Flight, ...]
    n_drones: int

    @property
    def makespan_s(self) -> float:
        return max((f.t_land for f in self.flights), default=0.0)

    def by_drone(self) -> dict[int, list[Flight]]:
        out: dict[int, list[Flight]] = {d: [] for d in range(self.n_drones)}
        for f in sorted(self.flights, key=lambda f: f.t_launch):
            out[f.drone].append(f)
        return out


@dataclass(frozen=True, eq=False)
class Body:
    xy: FloatArray
    rc: IntArray


def initial_stations(n_drones: int, n_stations: int) -> list[int]:
    return [d % n_stations for d in range(n_drones)]


def charge_duration_s(distance_m: float, cfg: PlannerConfig) -> float:
    return cfg.station.charge_time_s * min(1.0, distance_m / cfg.drone.range_m)
