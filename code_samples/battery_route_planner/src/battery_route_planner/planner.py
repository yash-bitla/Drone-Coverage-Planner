from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

import numpy as np
import numpy.typing as npt
from shapely.geometry import Polygon

from battery_route_planner.errors import InfeasibleRouteError, InvalidInputError
from battery_route_planner.router import Router

FloatArray = npt.NDArray[np.float64]
_EPS = 1e-9


@dataclass(frozen=True, slots=True)
class Span:
    """Inclusive route-point indices covered by one sortie."""

    start: int
    end: int


@dataclass(frozen=True, slots=True)
class SplitInput:
    """Precomputed distances needed by the dynamic program."""

    prefix: FloatArray
    near: FloatArray
    nearest_station: npt.NDArray[np.int64]

    @classmethod
    def from_route(
        cls,
        route_xy: FloatArray,
        stations_xy: FloatArray,
        router: Router,
    ) -> "SplitInput":
        if len(route_xy) == 0:
            raise InvalidInputError("route must contain at least one point")

        if len(route_xy) == 1:
            steps = np.empty(0, dtype=np.float64)
        else:
            steps = router.pair_distances(route_xy[:-1], route_xy[1:])
            if not np.isfinite(steps).all():
                raise InfeasibleRouteError("at least one consecutive route segment is unreachable")

        near, nearest_station = router.nearest(route_xy, stations_xy)
        if not np.isfinite(near).all():
            raise InfeasibleRouteError("at least one route point cannot reach any station")

        return cls(
            prefix=np.concatenate([[0.0], np.cumsum(steps)]),
            near=near,
            nearest_station=nearest_station,
        )

    def route_distance(self, span: Span) -> float:
        return float(self.prefix[span.end] - self.prefix[span.start])

    def sortie_distance(self, span: Span) -> float:
        return float(
            self.near[span.start]
            + self.route_distance(span)
            + self.near[span.end]
        )

    def check_reachable(self, budget_m: float) -> None:
        unreachable = int((2.0 * self.near > budget_m + _EPS).sum())
        if unreachable:
            raise InfeasibleRouteError(
                f"{unreachable} route point(s) are beyond half the battery range from every station"
            )


@dataclass(frozen=True, slots=True)
class Sortie:
    """One battery-feasible flight covering a contiguous slice of the route."""

    start_index: int
    end_index: int
    start_station: int
    end_station: int
    route_distance_m: float
    station_distance_m: float
    total_distance_m: float
    path: tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class PlanResult:
    """Result returned by :func:`plan_sorties`."""

    sorties: tuple[Sortie, ...]
    total_distance_m: float

    @property
    def sortie_count(self) -> int:
        return len(self.sorties)


def split_route(inp: SplitInput, battery_range_m: float) -> list[Span]:
    """Find the minimum-total-distance feasible split for a fixed route order.

    Let ``f[j]`` be the minimum flight distance needed to cover route points
    ``0..j-1``. For every possible final sortie ``i..j``, we relax ``f[j+1]``
    when the sortie fits within the battery range.

    The algorithm is O(n^2) after routing distances have been precomputed.
    """
    if battery_range_m <= 0:
        raise InvalidInputError("battery_range_m must be > 0")

    point_count = len(inp.near)
    if point_count == 0:
        return []

    inp.check_reachable(battery_range_m)

    best = np.full(point_count + 1, np.inf, dtype=np.float64)
    predecessor = np.full(point_count + 1, -1, dtype=np.int64)
    best[0] = 0.0

    for end in range(point_count):
        start = end
        while start >= 0:
            along = inp.prefix[end] - inp.prefix[start]
            if along > battery_range_m + _EPS:
                break

            if np.isfinite(best[start]):
                cost = inp.near[start] + along + inp.near[end]
                if cost <= battery_range_m + _EPS and best[start] + cost < best[end + 1]:
                    best[end + 1] = best[start] + cost
                    predecessor[end + 1] = start

            start -= 1

    if not np.isfinite(best[-1]):
        raise InfeasibleRouteError(
            "the route cannot be partitioned into battery-feasible sorties"
        )

    spans: list[Span] = []
    cursor = point_count
    while cursor > 0:
        start = int(predecessor[cursor])
        if start < 0:
            raise InfeasibleRouteError("failed to reconstruct a feasible route split")
        spans.append(Span(start, cursor - 1))
        cursor = start

    spans.reverse()
    return spans


def plan_sorties(
    route: npt.ArrayLike,
    stations: npt.ArrayLike,
    battery_range_m: float,
    obstacles: Sequence[Polygon] = (),
) -> PlanResult:
    """Split an ordered route into minimum-distance battery-feasible sorties.

    All movement is obstacle-aware: station travel and movement between
    consecutive route waypoints. The route order itself is fixed; this function
    optimizes only where to cut that route into sorties.
    """
    route_xy = _as_xy(route, "route")
    stations_xy = _as_xy(stations, "stations")

    if len(route_xy) == 0:
        raise InvalidInputError("route must contain at least one point")
    if len(stations_xy) == 0:
        raise InvalidInputError("at least one charging station is required")
    if battery_range_m <= 0:
        raise InvalidInputError("battery_range_m must be > 0")

    router = Router(obstacles)

    if router.contains(route_xy).any():
        raise InvalidInputError("a route point lies inside an obstacle")
    if router.contains(stations_xy).any():
        raise InvalidInputError("a charging station lies inside an obstacle")

    inp = SplitInput.from_route(route_xy, stations_xy, router)
    spans = split_route(inp, battery_range_m)

    sorties: list[Sortie] = []
    for span in spans:
        start_station = int(inp.nearest_station[span.start])
        end_station = int(inp.nearest_station[span.end])

        route_distance = inp.route_distance(span)
        station_distance = float(inp.near[span.start] + inp.near[span.end])
        total_distance = route_distance + station_distance

        path = _sortie_path(
            route_xy,
            stations_xy,
            span,
            start_station,
            end_station,
            router,
        )

        sorties.append(
            Sortie(
                start_index=span.start,
                end_index=span.end,
                start_station=start_station,
                end_station=end_station,
                route_distance_m=route_distance,
                station_distance_m=station_distance,
                total_distance_m=total_distance,
                path=tuple((float(x), float(y)) for x, y in path),
            )
        )

    return PlanResult(
        sorties=tuple(sorties),
        total_distance_m=float(sum(s.total_distance_m for s in sorties)),
    )


def _sortie_path(
    route_xy: FloatArray,
    stations_xy: FloatArray,
    span: Span,
    start_station: int,
    end_station: int,
    router: Router,
) -> FloatArray:
    pieces = [router.path(stations_xy[start_station], route_xy[span.start])]

    for i, j in pairwise(range(span.start, span.end + 1)):
        pieces.append(router.path(route_xy[i], route_xy[j]))

    pieces.append(router.path(route_xy[span.end], stations_xy[end_station]))

    out = pieces[0]
    for piece in pieces[1:]:
        out = np.vstack([out, piece[1:]])
    return out


def _as_xy(points: npt.ArrayLike, name: str) -> FloatArray:
    arr = np.asarray(points, dtype=np.float64)
    try:
        return arr.reshape(-1, 2)
    except ValueError as exc:
        raise InvalidInputError(f"{name} must be shaped as (n, 2)") from exc
