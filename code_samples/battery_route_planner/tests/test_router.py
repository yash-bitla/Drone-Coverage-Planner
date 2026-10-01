import math

import numpy as np
import pytest
from shapely.geometry import box

from battery_route_planner import Router

WALL = box(40.0, -10.0, 60.0, 10.0)
A = np.array([0.0, 0.0])
B = np.array([100.0, 0.0])


def test_no_obstacles_returns_straight_path() -> None:
    router = Router()

    assert router.path(A, B).tolist() == [[0.0, 0.0], [100.0, 0.0]]
    assert router.pair_distances([A], [B])[0] == pytest.approx(100.0)


def test_detours_around_obstacle_corners() -> None:
    router = Router([WALL])

    path = router.path(A, B)
    segment_lengths = np.hypot(*(path[1:] - path[:-1]).T)

    assert segment_lengths.sum() == pytest.approx(2 * math.hypot(40, 10) + 20)
    assert router.clear(path[:-1], path[1:]).all()


def test_nearest_uses_obstacle_aware_distance() -> None:
    router = Router([WALL])
    points = np.array([[100.0, 0.0]])
    stations = np.array([[0.0, 0.0], [100.0, 30.0]])

    distance, station = router.nearest(points, stations)

    assert station.tolist() == [1]
    assert distance[0] == pytest.approx(30.0)
