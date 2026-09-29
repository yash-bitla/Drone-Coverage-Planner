from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from droneplan._types import BoolArray, FloatArray, IntArray
from droneplan.config import DroneSpec
from droneplan.coverage import _kernels
from droneplan.coverage.lanes import Lane, extract_lanes
from droneplan.errors import InvalidInputError
from droneplan.geometry.grid import Grid
from droneplan.geometry.routing import Router
from droneplan.kinematics import turn_loss_s

_log = logging.getLogger(__name__)
_MAX_ROUTED_PAIRS = 4_000_000


@dataclass(frozen=True, eq=False)
class Tour:
    rc: IntArray
    xy: FloatArray

    def __len__(self) -> int:
        return len(self.xy)

    @property
    def n_cells(self) -> int:
        return int((self.rc[:, 0] >= 0).sum())


def lane_ends(grid: Grid, lanes: list[Lane]) -> FloatArray:
    rc = np.array([[ln.row, col] for ln in lanes for col in (ln.col_start, ln.col_end)])
    return grid.centers(rc)


def transition_costs(ends: FloatArray, router: Router, drone: DroneSpec) -> FloatArray:
    """cost[x, e] in metres-equivalent: route length plus turn time at cruise speed, from
    leaving lane end x to entering lane end e.

    Leaving at a left end means the lane was flown right to left (heading pi); entering at a
    left end means flying left to right (heading 0).
    """
    n = len(ends)
    left = np.arange(n) % 2 == 0
    exit_heading = np.where(left, np.pi, 0.0)
    entry_heading = np.where(left, 0.0, np.pi)
    x, e = np.divmod(np.arange(n * n), n)
    if router.has_obstacles and n * n <= _MAX_ROUTED_PAIRS:
        dist, first, last = router.routes(ends[x], ends[e])
    else:
        if router.has_obstacles:
            _log.warning(
                "%d lanes: ordering on straight-line costs; paths still avoid obstacles", n // 2
            )
        d = ends[e] - ends[x]
        dist = np.hypot(d[:, 0], d[:, 1])
        first = last = np.arctan2(d[:, 1], d[:, 0])

    def turn_m(h0: FloatArray, h1: FloatArray) -> FloatArray:
        return turn_loss_s(drone.speed_mps, drone.accel_mps2, h1 - h0) * drone.speed_mps

    cost = dist + turn_m(exit_heading[x], first) + turn_m(last, entry_heading[e])
    same_point = dist < 1e-9
    cost[same_point] = turn_m(exit_heading[x], entry_heading[e])[same_point]
    cost = cost.reshape(n, n)
    return (cost + cost.T) / 2


def build_tour(
    grid: Grid,
    stations_xy: FloatArray,
    drone: DroneSpec,
    router: Router,
    *,
    mask: BoolArray | None = None,
    max_passes: int = 50,
) -> Tour:
    lanes = extract_lanes(grid.required if mask is None else mask)
    if not lanes:
        raise InvalidInputError("nothing to cover")
    ends = lane_ends(grid, lanes)
    start, _ = router.nearest(ends, stations_xy)
    cost = transition_costs(ends, router, drone)
    order, orient = _kernels.nearest_neighbour(cost, start)
    _kernels.two_opt(order, orient, cost, max_passes)

    chunks = []
    for lane_idx, o in zip(order.tolist(), orient.tolist(), strict=True):
        lane = lanes[lane_idx]
        cols = np.arange(lane.col_start, lane.col_end + 1)
        chunks.append(np.column_stack([np.full(len(cols), lane.row), cols[::-1] if o else cols]))
    rc = np.concatenate(chunks).astype(np.int64)
    xy, rc = router.connect(grid.centers(rc), rc)
    return Tour(rc=rc, xy=xy)
