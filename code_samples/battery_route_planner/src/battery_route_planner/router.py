from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import numpy.typing as npt
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

from battery_route_planner.errors import InfeasibleRouteError, InvalidInputError

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int64]
BoolArray = npt.NDArray[np.bool_]

_TOUCH_TOLERANCE_M = 0.01
_CHUNK_ELEMENTS = 4_000_000


def _as_xy(points: npt.ArrayLike) -> FloatArray:
    arr = np.asarray(points, dtype=np.float64)
    try:
        return arr.reshape(-1, 2)
    except ValueError as exc:
        raise InvalidInputError("points must be shaped as (n, 2)") from exc


def _convex_vertices(obstacles: Sequence[Polygon]) -> FloatArray:
    """Keep only obstacle vertices where a Euclidean shortest path can bend."""
    kept: list[FloatArray] = []
    for poly in obstacles:
        ring = np.asarray(orient(poly, 1.0).exterior.coords, dtype=np.float64)[:-1]
        before, after = np.roll(ring, 1, axis=0), np.roll(ring, -1, axis=0)
        cross = (ring[:, 0] - before[:, 0]) * (after[:, 1] - ring[:, 1]) - (
            ring[:, 1] - before[:, 1]
        ) * (after[:, 0] - ring[:, 0])
        kept.append(ring[cross > 1e-9])
    return np.vstack(kept) if kept else np.empty((0, 2), dtype=np.float64)


class Router:
    """Shortest obstacle-avoiding routes in a 2-D metric coordinate system.

    The router builds a visibility graph over convex obstacle vertices. Query
    points are connected to the visible graph vertices only when a direct path
    is blocked; otherwise the Euclidean segment is returned immediately.
    """

    def __init__(self, obstacles: Sequence[Polygon] = ()) -> None:
        merged = (
            [
                part
                for part in shapely.get_parts(shapely.union_all(obstacles))
                if part.geom_type == "Polygon" and part.area > 0
            ]
            if obstacles
            else []
        )
        self.obstacles = tuple(merged)
        blockers = [o.buffer(-_TOUCH_TOLERANCE_M) for o in self.obstacles]
        blockers = [b for b in blockers if not b.is_empty]
        self._tree = shapely.STRtree(blockers) if blockers else None
        self._boxes = np.array([b.bounds for b in blockers], dtype=np.float64).reshape(-1, 4)
        self._union = shapely.MultiPolygon(self.obstacles) if self.obstacles else None
        self.nodes = _convex_vertices(self.obstacles)
        self._dist, self._pred = self._all_pairs()

    @property
    def has_obstacles(self) -> bool:
        return self._tree is not None

    def contains(self, points: npt.ArrayLike) -> BoolArray:
        pts = _as_xy(points)
        if self._union is None:
            return np.zeros(len(pts), dtype=bool)
        return np.asarray(
            shapely.contains_xy(self._union, pts[:, 0], pts[:, 1]),
            dtype=bool,
        )

    def clear(self, a: npt.ArrayLike, b: npt.ArrayLike) -> BoolArray:
        """Return whether each a[i] -> b[i] segment avoids obstacle interiors."""
        a_xy, b_xy = _as_xy(a), _as_xy(b)
        if len(a_xy) != len(b_xy):
            raise InvalidInputError("a and b must contain the same number of points")

        clear = np.ones(len(a_xy), dtype=bool)
        if self._tree is None or len(a_xy) == 0:
            return clear

        lo, hi = np.minimum(a_xy, b_xy), np.maximum(a_xy, b_xy)
        boxes = self._boxes
        near_box = (
            (lo[:, None, 0] <= boxes[None, :, 2])
            & (hi[:, None, 0] >= boxes[None, :, 0])
            & (lo[:, None, 1] <= boxes[None, :, 3])
            & (hi[:, None, 1] >= boxes[None, :, 1])
        ).any(axis=1)

        idx = np.flatnonzero(near_box)
        if len(idx):
            segments = shapely.linestrings(np.stack([a_xy[idx], b_xy[idx]], axis=1))
            hits = self._tree.query(segments, predicate="intersects")[0]
            clear[idx[np.unique(hits)]] = False
        return clear

    def pair_distances(self, a: npt.ArrayLike, b: npt.ArrayLike) -> FloatArray:
        """Obstacle-aware distance for each paired a[i] -> b[i] query."""
        a_xy, b_xy = _as_xy(a), _as_xy(b)
        if len(a_xy) != len(b_xy):
            raise InvalidInputError("a and b must contain the same number of points")

        delta = b_xy - a_xy
        distances = np.hypot(delta[:, 0], delta[:, 1])
        blocked = np.flatnonzero(~self.clear(a_xy, b_xy))
        if len(blocked) == 0:
            return distances
        if len(self.nodes) == 0:
            distances[blocked] = np.inf
            return distances

        vertex_count = len(self.nodes)
        chunk = max(1, _CHUNK_ELEMENTS // max(vertex_count * vertex_count, 1))
        for start in range(0, len(blocked), chunk):
            rows = blocked[start : start + chunk]
            from_a = self._to_nodes(a_xy[rows])
            to_b = self._to_nodes(b_xy[rows])
            via = np.min(self._dist[None, :, :] + to_b[:, None, :], axis=2)
            distances[rows] = np.min(from_a + via, axis=1)
        return distances

    def distances(self, points: npt.ArrayLike, targets: npt.ArrayLike) -> FloatArray:
        """Matrix of obstacle-aware distances from every point to every target."""
        pts, dst = _as_xy(points), _as_xy(targets)
        if len(dst) == 0:
            raise InvalidInputError("at least one target is required")
        pi = np.repeat(np.arange(len(pts)), len(dst))
        ti = np.tile(np.arange(len(dst)), len(pts))
        return self.pair_distances(pts[pi], dst[ti]).reshape(len(pts), len(dst))

    def nearest(
        self,
        points: npt.ArrayLike,
        targets: npt.ArrayLike,
    ) -> tuple[FloatArray, IntArray]:
        """Distance and index of the nearest obstacle-aware target for each point."""
        matrix = self.distances(points, targets)
        idx = np.argmin(matrix, axis=1)
        return matrix[np.arange(len(matrix)), idx], idx.astype(np.int64)

    def path(self, start: npt.ArrayLike, end: npt.ArrayLike) -> FloatArray:
        """Return one shortest obstacle-free polyline from start to end."""
        p, q = _as_xy(start)[0], _as_xy(end)[0]
        if self.clear(p, q)[0]:
            return np.array([p, q], dtype=np.float64)

        if len(self.nodes) == 0:
            raise InfeasibleRouteError(f"no obstacle-free route from {p.tolist()} to {q.tolist()}")

        total = (
            self._to_nodes(p[None])[0][:, None]
            + self._dist
            + self._to_nodes(q[None])[0][None, :]
        )
        a, b = np.unravel_index(np.argmin(total), total.shape)
        if not np.isfinite(total[a, b]):
            raise InfeasibleRouteError(f"no obstacle-free route from {p.tolist()} to {q.tolist()}")

        chain = [int(b)]
        while chain[-1] != a:
            predecessor = int(self._pred[a, chain[-1]])
            if predecessor < 0:
                raise InfeasibleRouteError(
                    f"no obstacle-free route from {p.tolist()} to {q.tolist()}"
                )
            chain.append(predecessor)

        return np.vstack([p, self.nodes[chain[::-1]], q])

    def _all_pairs(self) -> tuple[FloatArray, IntArray]:
        vertex_count = len(self.nodes)
        if vertex_count == 0:
            return (
                np.empty((0, 0), dtype=np.float64),
                np.empty((0, 0), dtype=np.int64),
            )

        i, j = np.triu_indices(vertex_count, 1)
        visible = self.clear(self.nodes[i], self.nodes[j])
        weights = np.hypot(*(self.nodes[j[visible]] - self.nodes[i[visible]]).T)
        graph = csr_matrix(
            (weights, (i[visible], j[visible])),
            shape=(vertex_count, vertex_count),
        )
        dist, pred = shortest_path(
            graph,
            method="D",
            directed=False,
            return_predecessors=True,
        )
        return dist, pred.astype(np.int64)

    def _to_nodes(self, points: FloatArray) -> FloatArray:
        point_count, vertex_count = len(points), len(self.nodes)
        if vertex_count == 0:
            return np.empty((point_count, 0), dtype=np.float64)

        pi = np.repeat(np.arange(point_count), vertex_count)
        ni = np.tile(np.arange(vertex_count), point_count)
        distances = np.hypot(*(self.nodes[ni] - points[pi]).T)
        distances[~self.clear(points[pi], self.nodes[ni])] = np.inf
        return distances.reshape(point_count, vertex_count)
