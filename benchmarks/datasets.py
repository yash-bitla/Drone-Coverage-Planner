from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.geometry.base import BaseGeometry

from droneplan.config import DroneSpec, PlannerConfig, StationSpec
from droneplan.geometry.frame import Frame, utm_crs_for
from droneplan.geometry.obstacles import Obstacle

BASE_LNGLAT = (78.0, 20.0)
_FRAME = Frame(utm_crs_for(*BASE_LNGLAT))
_ORIGIN = _FRAME.lnglat_to_utm(np.array([BASE_LNGLAT]))[0]


@dataclass(frozen=True, eq=False)
class Instance:
    name: str
    area: BaseGeometry
    stations: np.ndarray
    obstacles: list[Obstacle]
    config: PlannerConfig


def _to_lnglat(geom: BaseGeometry) -> BaseGeometry:
    return shapely.transform(geom, lambda xy: _FRAME.utm_to_lnglat(xy + _ORIGIN))


def _random_point(rng: np.random.Generator, geom: BaseGeometry) -> Point:
    minx, miny, maxx, maxy = geom.bounds
    while True:
        p = Point(rng.uniform(minx, maxx), rng.uniform(miny, maxy))
        if geom.contains(p):
            return p


def synthetic_instance(seed: int, *, with_obstacles: bool = False) -> Instance:
    rng = np.random.default_rng(seed)
    n_v = int(rng.integers(8, 16))
    angles = np.sort(rng.uniform(0, 2 * np.pi, n_v))
    radii = rng.uniform(0.6, 1.0, n_v) * rng.uniform(1200, 3000)
    stretch = rng.uniform(1.0, 3.0)
    area = Polygon(np.column_stack([np.cos(angles) * radii * stretch, np.sin(angles) * radii]))
    area = affinity.rotate(area.buffer(0), rng.uniform(0, 180), origin=(0, 0))
    for _ in range(int(rng.integers(0, 3))):
        area = area.difference(_random_point(rng, area).buffer(rng.uniform(100, 300)))

    stations = [_random_point(rng, area.buffer(200)) for _ in range(int(rng.integers(1, 5)))]
    obstacles: list[Obstacle] = []
    if with_obstacles:
        while len(obstacles) < int(rng.integers(1, 5)):
            c = _random_point(rng, area)
            w, h = rng.uniform(60, 400), rng.uniform(60, 400)
            rect = affinity.rotate(
                box(c.x - w / 2, c.y - h / 2, c.x + w / 2, c.y + h / 2), rng.uniform(0, 180)
            )
            if min(rect.distance(s) for s in stations) > 150:
                obstacles.append(Obstacle(_to_lnglat(rect), float(rng.uniform(30, 250))))

    st_xy = np.array([[s.x, s.y] for s in stations])
    boundary = np.vstack([np.asarray(p.exterior.coords) for p in shapely.get_parts(area)])
    far = float(
        np.max(
            np.min(
                np.hypot(
                    boundary[:, None, 0] - st_xy[None, :, 0],
                    boundary[:, None, 1] - st_xy[None, :, 1],
                ),
                axis=1,
            )
        )
    )
    reserve = 0.10
    range_m = max(float(rng.uniform(6_000, 20_000)), 2.6 * far / (1 - reserve))
    cfg = PlannerConfig(
        drone=DroneSpec(
            count=int(rng.integers(2, 9)),
            range_m=range_m,
            speed_mps=float(rng.uniform(10, 18)),
            reserve_frac=reserve,
        ),
        station=StationSpec(charge_time_s=float(rng.choice([900.0, 1800.0, 2700.0]))),
    )
    suffix = "-obs" if with_obstacles else ""
    return Instance(
        f"syn-{seed:03d}{suffix}",
        _to_lnglat(area),
        _FRAME.utm_to_lnglat(st_xy + _ORIGIN),
        obstacles,
        cfg,
    )
