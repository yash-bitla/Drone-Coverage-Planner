import math

import numpy as np
import pytest
from shapely.geometry import box

from droneplan.errors import InvalidInputError
from droneplan.geometry.routing import Router
from droneplan.kinematics import polyline_length

WALL = box(40, -10, 60, 10)
A, B = np.array([0.0, 0.0]), np.array([100.0, 0.0])


def test_no_obstacles_is_straight() -> None:
    r = Router()
    assert r.path(A, B).tolist() == [[0.0, 0.0], [100.0, 0.0]]
    assert r.distances(A, B)[0, 0] == pytest.approx(100.0)


def test_detours_around_obstacle_corners() -> None:
    r = Router([WALL])
    path = r.path(A, B)
    assert polyline_length(path) == pytest.approx(2 * math.hypot(40, 10) + 20)
    assert r.clear(path[:-1], path[1:]).all()
    assert r.distances(A, B)[0, 0] == pytest.approx(polyline_length(path))


def test_routes_report_leg_headings() -> None:
    _, first, last = Router([WALL]).routes(A[None], B[None])
    assert abs(first[0]) == pytest.approx(math.atan2(10, 40))
    assert abs(last[0]) == pytest.approx(math.atan2(10, 40))
    assert np.sign(first[0]) == -np.sign(last[0])


def test_touching_an_edge_is_allowed() -> None:
    assert Router([WALL]).clear([[40.0, 10.0]], [[60.0, 10.0]]).all()


def test_nearest_matches_full_distances() -> None:
    r = Router([WALL, box(120, 30, 140, 90)])
    rng = np.random.default_rng(0)
    pts = rng.uniform(-50, 200, (200, 2))
    pts = pts[~r.contains(pts)]
    stations = np.array([[100.0, 0.0], [0.0, 90.0], [180.0, 60.0]])
    d, i = r.nearest(pts, stations)
    full = r.distances(pts, stations)
    np.testing.assert_allclose(d, full.min(axis=1))
    np.testing.assert_array_equal(i, full.argmin(axis=1))
    with pytest.raises(InvalidInputError):
        r.nearest(pts, np.empty((0, 2)))


def test_connect_inserts_waypoints() -> None:
    xy, rc = Router([WALL]).connect(np.array([A, B]), np.array([[0, 0], [0, 1]]))
    assert len(xy) == 4
    assert rc[1:3].tolist() == [[-1, -1], [-1, -1]]


def test_contains() -> None:
    assert Router([WALL]).contains([[50.0, 0.0], [0.0, 0.0]]).tolist() == [True, False]


def test_merges_touching_obstacles() -> None:
    r = Router([box(0, 0, 10, 10), box(10, 0, 20, 10)])
    path = r.path([10, -5], [10, 15])
    assert path.tolist() != [[10.0, -5.0], [10.0, 15.0]]
    assert r.clear(path[:-1], path[1:]).all()
    assert polyline_length(path) > 20.0
