import math

import numpy as np
import pytest

from droneplan.kinematics import (
    count_turns,
    heading_change,
    polyline_length,
    polyline_time,
    vertex_times,
)


def pts(*p: tuple[float, float]) -> np.ndarray:
    return np.array(p, dtype=np.float64)


def test_straight_line() -> None:
    assert polyline_time(pts((0, 0), (100, 0)), 10, 2) == pytest.approx(10.0)


def test_collinear_vertices_cost_nothing() -> None:
    assert polyline_time(pts((0, 0), (50, 0), (100, 0)), 10, 2) == pytest.approx(10.0)


def test_u_turn_loses_v_over_a() -> None:
    assert polyline_time(pts((0, 0), (100, 0), (0, 0)), 10, 2) == pytest.approx(25.0)


def test_right_angle_loses_half() -> None:
    assert polyline_time(pts((0, 0), (100, 0), (100, 100)), 10, 2) == pytest.approx(22.5)


def test_duplicate_vertices_are_ignored() -> None:
    np.testing.assert_allclose(vertex_times(pts((0, 0), (0, 0), (100, 0)), 10, 2), [0, 0, 10])


def test_length_and_turns() -> None:
    square = pts((0, 0), (10, 0), (10, 10), (0, 10), (0, 0))
    assert polyline_length(square) == pytest.approx(40.0)
    assert count_turns(square) == 3


def test_heading_change_wraps() -> None:
    assert float(heading_change(3.1, -3.1)) == pytest.approx(2 * math.pi - 6.2)


def test_degenerate_inputs() -> None:
    assert polyline_time(pts((1, 1)), 10, 2) == 0.0
    assert polyline_length(np.empty((0, 2))) == 0.0
