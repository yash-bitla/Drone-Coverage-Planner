from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import numpy.typing as npt
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

from droneplan._types import BoolArray, FloatArray, IntArray
from droneplan.errors import InfeasiblePlanError, InvalidInputError

_TOUCH_TOLERANCE_M = 0.01
_CHUNK_ELEMENTS = 4_000_000


def _xy(a: npt.ArrayLike) -> FloatArray:
    return np.asarray(a, dtype=np.float64).reshape(-1, 2)


def _convex_vertices(obstacles: Sequence[Polygon]) -> FloatArray:
    kept = []
    for poly in obstacles:
        ring = np.asarray(orient(poly, 1.0).exterior.coords)[:-1]
        before, after = np.roll(ring, 1, axis=0), np.roll(ring, -1, axis=0)
        cross = (ring[:, 0] - before[:, 0]) * (after[:, 1] - ring[:, 1]) - (
            ring[:, 1] - before[:, 1]
        ) * (after[:, 0] - ring[:, 0])
        kept.append(ring[cross > 1e-9])
    return np.vstack(kept) if kept else np.empty((0, 2))


class Router:
    """Shortest obstacle-avoiding routes in the plan frame.

    A visibility graph over convex obstacle vertices, the only points where a shortest path
    among polygons can bend. Segments may run along an obstacle's boundary but not enter it.
    Without obstacles every route is a straight line.
    """

    def __init__(self, obstacles: Sequence[Polygon] = ()) -> None:
        self.obstacles = tuple(obstacles)
        blockers = [o.buffer(-_TOUCH_TOLERANCE_M) for o in self.obstacles]
        self._tree = shapely.STRtree(blockers) if blockers else None
        self._boxes = np.array([b.bounds for b in blockers], dtype=np.float64).reshape(-1, 4)
        self._union = shapely.union_all(self.obstacles) if self.obstacles else None
        self.nodes = _convex_vertices(self.obstacles)
        self._dist, self._pred = self._all_pairs()

    @property
    def has_obstacles(self) -> bool:
        return self._tree is not None

    def contains(self, points: npt.ArrayLike) -> BoolArray:
        p = _xy(points)
        if self._union is None:
            return np.zeros(len(p), dtype=bool)
        return np.asarray(shapely.contains_xy(self._union, p[:, 0], p[:, 1]), dtype=bool)

    def clear(self, a: npt.ArrayLike, b: npt.ArrayLike) -> BoolArray:
        a, b = _xy(a), _xy(b)
        out = np.ones(len(a), dtype=bool)
        if self._tree is None or len(a) == 0:
            return out
        lo, hi = np.minimum(a, b), np.maximum(a, b)
        bx = self._boxes
        near_box = (
            (lo[:, None, 0] <= bx[None, :, 2])
            & (hi[:, None, 0] >= bx[None, :, 0])
            & (lo[:, None, 1] <= bx[None, :, 3])
            & (hi[:, None, 1] >= bx[None, :, 1])
        ).any(axis=1)
        idx = np.flatnonzero(near_box)
        if len(idx):
            segments = shapely.linestrings(np.stack([a[idx], b[idx]], axis=1))
            hits = self._tree.query(segments, predicate="intersects")[0]
            out[idx[np.unique(hits)]] = False
        return out

    def routes(
        self, a: npt.ArrayLike, b: npt.ArrayLike
    ) -> tuple[FloatArray, FloatArray, FloatArray]:
        """Pairwise a[i] -> b[i]: (distance, heading of the first leg, heading of the last leg)."""
        a, b = _xy(a), _xy(b)
        d = b - a
        dist = np.hypot(d[:, 0], d[:, 1])
        first = np.arctan2(d[:, 1], d[:, 0])
        last = first.copy()
        blocked = np.flatnonzero(~self.clear(a, b))
        dist[blocked] = np.inf
        v = len(self.nodes)
        if len(blocked) == 0 or v == 0:
            return dist, first, last
        chunk = max(1, _CHUNK_ELEMENTS // (v * v))
        for start in range(0, len(blocked), chunk):
            rows = blocked[start : start + chunk]
            from_a, to_b = self._to_nodes(a[rows]), self._to_nodes(b[rows])
            via = np.min(self._dist[None, :, :] + to_b[:, None, :], axis=2)
            total = from_a + via
            ai = np.argmin(total, axis=1)
            bi = np.argmin(self._dist[ai] + to_b, axis=1)
            dist[rows] = total[np.arange(len(rows)), ai]
            lead = self.nodes[ai] - a[rows]
            tail = b[rows] - self.nodes[bi]
            first[rows] = np.arctan2(lead[:, 1], lead[:, 0])
            last[rows] = np.arctan2(tail[:, 1], tail[:, 0])
        return dist, first, last

    def distances(self, points: npt.ArrayLike, targets: npt.ArrayLike) -> FloatArray:
        p, t = _xy(points), _xy(targets)
        pi, ti = np.repeat(np.arange(len(p)), len(t)), np.tile(np.arange(len(t)), len(p))
        return self.routes(p[pi], t[ti])[0].reshape(len(p), len(t))

    def nearest(self, points: npt.ArrayLike, targets: npt.ArrayLike) -> tuple[FloatArray, IntArray]:
        p, t = _xy(points), _xy(targets)
        if len(t) == 0:
            raise InvalidInputError("at least one station is required")
        dist = np.hypot(p[:, None, 0] - t[None, :, 0], p[:, None, 1] - t[None, :, 1])
        if self.has_obstacles:
            pi, ti = np.divmod(np.arange(dist.size), len(t))
            blocked = ~self.clear(p[pi], t[ti]).reshape(dist.shape)
            straight = np.where(blocked, np.inf, dist)
            # A detour is never shorter than the straight line, so only blocked targets closer
            # than the best clear one can change the answer.
            r, c = np.nonzero(blocked & (dist < straight.min(axis=1, keepdims=True)))
            if len(r):
                straight[r, c] = self.routes(p[r], t[c])[0]
            dist = straight
        idx = np.argmin(dist, axis=1)
        return dist[np.arange(len(p)), idx], idx.astype(np.int64)

    def path(self, p: npt.ArrayLike, q: npt.ArrayLike) -> FloatArray:
        p0, q0 = _xy(p)[0], _xy(q)[0]
        if self.clear(p0, q0)[0]:
            return np.array([p0, q0])
        total = (
            self._to_nodes(p0[None])[0][:, None] + self._dist + self._to_nodes(q0[None])[0][None, :]
        )
        a, b = np.unravel_index(np.argmin(total), total.shape) if total.size else (0, 0)
        if not total.size or not np.isfinite(total[a, b]):
            raise InfeasiblePlanError(f"no obstacle-free route from {p0.tolist()} to {q0.tolist()}")
        chain = [int(b)]
        while chain[-1] != a:
            chain.append(int(self._pred[a, chain[-1]]))
        return np.vstack([p0, self.nodes[chain[::-1]], q0])

    def connect(self, xy: FloatArray, rc: IntArray) -> tuple[FloatArray, IntArray]:
        if not self.has_obstacles or len(xy) < 2:
            return xy, rc
        blocked = np.flatnonzero(~self.clear(xy[:-1], xy[1:]))
        if len(blocked) == 0:
            return xy, rc
        out_xy, out_rc, last = [], [], 0
        for i in blocked:
            detour = self.path(xy[i], xy[i + 1])[1:-1]
            out_xy += [xy[last : i + 1], detour]
            out_rc += [rc[last : i + 1], np.full((len(detour), 2), -1, dtype=np.int64)]
            last = int(i) + 1
        out_xy.append(xy[last:])
        out_rc.append(rc[last:])
        return np.vstack(out_xy), np.vstack(out_rc).astype(np.int64)

    def _all_pairs(self) -> tuple[FloatArray, IntArray]:
        v = len(self.nodes)
        if v == 0:
            return np.empty((0, 0)), np.empty((0, 0), dtype=np.int64)
        i, j = np.triu_indices(v, 1)
        ok = self.clear(self.nodes[i], self.nodes[j])
        w = np.hypot(*(self.nodes[j[ok]] - self.nodes[i[ok]]).T)
        graph = csr_matrix((w, (i[ok], j[ok])), shape=(v, v))
        dist, pred = shortest_path(graph, method="D", directed=False, return_predecessors=True)
        return dist, pred.astype(np.int64)

    def _to_nodes(self, points: FloatArray) -> FloatArray:
        n, v = len(points), len(self.nodes)
        pi, ni = np.repeat(np.arange(n), v), np.tile(np.arange(v), n)
        d = np.hypot(*(self.nodes[ni] - points[pi]).T)
        d[~self.clear(points[pi], self.nodes[ni])] = np.inf
        return d.reshape(n, v)
