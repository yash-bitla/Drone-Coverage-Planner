from __future__ import annotations

import math
from dataclasses import dataclass, field

from droneplan.errors import InvalidInputError


@dataclass(frozen=True, slots=True)
class SensorSpec:
    altitude_m: float = 100.0
    fov_deg: float = 60.0
    overlap: float = 0.2

    def __post_init__(self) -> None:
        if self.altitude_m <= 0:
            raise InvalidInputError("altitude_m must be > 0")
        if not 0 < self.fov_deg < 180:
            raise InvalidInputError("fov_deg must be in (0, 180)")
        if not 0 <= self.overlap < 1:
            raise InvalidInputError("overlap must be in [0, 1)")

    @property
    def footprint_m(self) -> float:
        """Lane spacing: the camera's ground footprint width minus the side overlap."""
        return 2 * self.altitude_m * math.tan(math.radians(self.fov_deg) / 2) * (1 - self.overlap)


@dataclass(frozen=True, slots=True)
class DroneSpec:
    count: int
    range_m: float
    speed_mps: float
    accel_mps2: float = 2.0
    reserve_frac: float = 0.10

    def __post_init__(self) -> None:
        if self.count < 1:
            raise InvalidInputError("count must be >= 1")
        if self.range_m <= 0 or self.speed_mps <= 0 or self.accel_mps2 <= 0:
            raise InvalidInputError("range_m, speed_mps and accel_mps2 must be > 0")
        if not 0 <= self.reserve_frac < 1:
            raise InvalidInputError("reserve_frac must be in [0, 1)")

    @property
    def usable_range_m(self) -> float:
        return self.range_m * (1 - self.reserve_frac)


@dataclass(frozen=True, slots=True)
class StationSpec:
    charge_time_s: float = 1800.0
    capacity: int = 1

    def __post_init__(self) -> None:
        if self.charge_time_s < 0:
            raise InvalidInputError("charge_time_s must be >= 0")
        if self.capacity < 1:
            raise InvalidInputError("capacity must be >= 1")


@dataclass(frozen=True, slots=True)
class PlannerConfig:
    drone: DroneSpec
    station: StationSpec = field(default_factory=StationSpec)
    sensor: SensorSpec = field(default_factory=SensorSpec)
    clearance_m: float = 20.0
    max_cells: int = 1_000_000

    def __post_init__(self) -> None:
        if self.clearance_m < 0:
            raise InvalidInputError("clearance_m must be >= 0")
        if self.max_cells < 1:
            raise InvalidInputError("max_cells must be >= 1")
