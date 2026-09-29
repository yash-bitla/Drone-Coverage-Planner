from __future__ import annotations

from typing import Any

import numpy as np

from droneplan.errors import InvalidInputError
from droneplan.kinematics import vertex_times
from droneplan.planner import Plan


def _feature(geometry: dict[str, Any], **properties: Any) -> dict[str, Any]:
    return {"type": "Feature", "geometry": geometry, "properties": properties}


def plan_to_geojson(plan: Plan) -> dict[str, Any]:
    """Stations, inflated obstacles and flights; each flight carries one timestamp per vertex."""
    if plan.frame is None:
        raise InvalidInputError("plan has no geographic frame; build it with plan_area")
    frame, drone = plan.frame, plan.config.drone
    features = [
        _feature({"type": "Point", "coordinates": ll}, kind="station", station=i)
        for i, ll in enumerate(frame.to_lnglat(plan.stations_xy).round(7).tolist())
    ]
    for obstacle in plan.router.obstacles:
        ring = frame.to_lnglat(np.asarray(obstacle.exterior.coords)).round(7).tolist()
        features.append(_feature({"type": "Polygon", "coordinates": [ring]}, kind="obstacle"))
    for f in plan.schedule.flights:
        times = vertex_times(f.path, drone.speed_mps, drone.accel_mps2) + f.t_launch
        features.append(
            _feature(
                {"type": "LineString", "coordinates": frame.to_lnglat(f.path).round(7).tolist()},
                kind="flight",
                flight_kind=f.kind.value,
                drone=f.drone,
                launch_station=f.launch_station,
                land_station=f.land_station,
                t_launch=round(f.t_launch, 2),
                t_land=round(f.t_land, 2),
                charge_start=round(f.charge_start, 2),
                charge_end=round(f.charge_end, 2),
                distance_m=round(f.distance_m, 1),
                timestamps=times.round(2).tolist(),
            )
        )
    return {
        "type": "FeatureCollection",
        "features": features,
        "properties": {
            "algorithm": plan.algorithm.value,
            "metrics": plan.metrics().as_dict(),
            "config": {
                "drones": drone.count,
                "range_m": drone.range_m,
                "usable_range_m": drone.usable_range_m,
                "speed_mps": drone.speed_mps,
                "charge_time_s": plan.config.station.charge_time_s,
                "capacity": plan.config.station.capacity,
            },
        },
    }
