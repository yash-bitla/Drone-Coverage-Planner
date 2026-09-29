from __future__ import annotations

from enum import StrEnum
from typing import Any

from pydantic import BaseModel, Field
from shapely.errors import GeometryTypeError
from shapely.geometry import shape
from shapely.geometry.base import BaseGeometry

from droneplan.config import DroneSpec, PlannerConfig, SensorSpec, StationSpec
from droneplan.errors import InvalidInputError
from droneplan.geometry.obstacles import Obstacle
from droneplan.planner import Algorithm

_POLYGONAL = ("Polygon", "MultiPolygon")


def _polygonal(geojson: dict[str, Any], what: str) -> BaseGeometry:
    try:
        geom = shape(geojson)
    except (GeometryTypeError, KeyError, TypeError, ValueError) as exc:
        raise InvalidInputError(f"{what} is not valid GeoJSON geometry: {exc}") from exc
    if geom.geom_type not in _POLYGONAL or geom.is_empty:
        raise InvalidInputError(f"{what} must be a non-empty Polygon or MultiPolygon")
    return geom


class ObstacleIn(BaseModel):
    geometry: dict[str, Any]
    height_m: float | None = Field(default=None, ge=0)


class PlanRequest(BaseModel):
    area: dict[str, Any]
    stations: list[tuple[float, float]] = Field(min_length=1)
    obstacles: list[ObstacleIn] = Field(default_factory=list)
    drones: int = Field(ge=1, le=64)
    range_km: float = Field(gt=0)
    speed_mps: float = Field(gt=0)
    charge_min: float = Field(default=30.0, ge=0)
    capacity: int = Field(default=1, ge=1)
    altitude_m: float = Field(default=100.0, gt=0)
    fov_deg: float = Field(default=60.0, gt=0, lt=180)
    overlap: float = Field(default=0.2, ge=0, lt=1)
    clearance_m: float = Field(default=20.0, ge=0)
    algorithm: Algorithm = Algorithm.RSS

    def config(self) -> PlannerConfig:
        return PlannerConfig(
            drone=DroneSpec(
                count=self.drones, range_m=self.range_km * 1000, speed_mps=self.speed_mps
            ),
            station=StationSpec(charge_time_s=self.charge_min * 60, capacity=self.capacity),
            sensor=SensorSpec(
                altitude_m=self.altitude_m, fov_deg=self.fov_deg, overlap=self.overlap
            ),
            clearance_m=self.clearance_m,
        )

    def area_geometry(self) -> BaseGeometry:
        return _polygonal(self.area, "area")

    def obstacle_list(self) -> list[Obstacle]:
        return [
            Obstacle(_polygonal(o.geometry, f"obstacle {i}"), o.height_m)
            for i, o in enumerate(self.obstacles)
        ]


class JobStatus(StrEnum):
    PENDING = "pending"
    DONE = "done"
    FAILED = "failed"


class JobOut(BaseModel):
    id: str
    status: JobStatus
    result: dict[str, Any] | None = None
    error: str | None = None
