from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt

from droneplan._types import FloatArray

_MIN_SEGMENT_M = 1e-9


def heading_change(h0: npt.ArrayLike, h1: npt.ArrayLike) -> FloatArray:
    d = np.asarray(h1, dtype=np.float64) - np.asarray(h0, dtype=np.float64)
    return np.abs((d + np.pi) % (2 * np.pi) - np.pi)


def turn_loss_s(speed_mps: float, accel_mps2: float, dtheta: npt.ArrayLike) -> FloatArray:
    """Seconds lost versus cruising for a heading change of `dtheta` radians.

    A full stop and re-acceleration costs v/a seconds; scaling by (1 - cos dtheta) / 2 gives
    0 for straight flight, v/2a for a right angle and v/a for a reversal.
    """
    dtheta = np.asarray(dtheta, dtype=np.float64)
    return (speed_mps / accel_mps2) * (1.0 - np.cos(dtheta)) / 2.0


def segment_lengths(xy: FloatArray) -> FloatArray:
    d = np.diff(np.asarray(xy, dtype=np.float64), axis=0)
    return np.hypot(d[:, 0], d[:, 1])


def _segments(xy: FloatArray) -> tuple[FloatArray, FloatArray, npt.NDArray[np.intp]]:
    d = np.diff(np.asarray(xy, dtype=np.float64), axis=0)
    length = np.hypot(d[:, 0], d[:, 1])
    moving = np.flatnonzero(length > _MIN_SEGMENT_M)
    return length, np.arctan2(d[moving, 1], d[moving, 0]), moving


def polyline_length(xy: FloatArray) -> float:
    return float(segment_lengths(xy).sum())


def vertex_times(xy: FloatArray, speed_mps: float, accel_mps2: float) -> FloatArray:
    """Arrival time at every vertex; a turn's loss is added to the segment after it."""
    times = np.zeros(len(xy), dtype=np.float64)
    if len(xy) < 2:
        return times
    length, headings, moving = _segments(xy)
    seg_time = length / speed_mps
    if len(moving) > 1:
        dtheta = heading_change(headings[:-1], headings[1:])
        seg_time[moving[1:]] += turn_loss_s(speed_mps, accel_mps2, dtheta)
    times[1:] = np.cumsum(seg_time)
    return times


def polyline_time(xy: FloatArray, speed_mps: float, accel_mps2: float) -> float:
    return float(vertex_times(xy, speed_mps, accel_mps2)[-1]) if len(xy) else 0.0


def count_turns(xy: FloatArray, min_angle_rad: float = math.radians(1.0)) -> int:
    if len(xy) < 3:
        return 0
    _, headings, _ = _segments(xy)
    if len(headings) < 2:
        return 0
    return int((heading_change(headings[:-1], headings[1:]) > min_angle_rad).sum())
