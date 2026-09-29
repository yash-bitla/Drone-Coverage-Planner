from __future__ import annotations

from droneplan._types import FloatArray, IntArray
from droneplan.config import PlannerConfig
from droneplan.kinematics import polyline_length, polyline_time
from droneplan.scheduling.charging import ChargingNetwork
from droneplan.scheduling.model import Flight, FlightKind, charge_duration_s


def landing_estimate(
    path: FloatArray, t_launch: float, land: int, network: ChargingNetwork, cfg: PlannerConfig
) -> tuple[float, float, float, int]:
    t_land = t_launch + polyline_time(path, cfg.drone.speed_mps, cfg.drone.accel_mps2)
    duration = charge_duration_s(polyline_length(path), cfg)
    start, pad = network.earliest_slot(land, t_land, duration)
    return t_land, start, duration, pad


def fly(
    drone: int,
    kind: FlightKind,
    launch: int,
    land: int,
    t_launch: float,
    path: FloatArray,
    covered: IntArray,
    network: ChargingNetwork,
    cfg: PlannerConfig,
) -> Flight:
    t_land, start, duration, pad = landing_estimate(path, t_launch, land, network, cfg)
    network.reserve(land, pad, start, duration)
    return Flight(
        drone, kind, launch, land, t_launch, t_land, start, start + duration, path, covered
    )
