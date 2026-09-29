import numpy as np
from numba import njit

# Lane k has ends 2k (left) and 2k + 1 (right); orient 0 flies left to right.


@njit(cache=True)
def _entry(lane, orient):
    return 2 * lane + orient


@njit(cache=True)
def _exit(lane, orient):
    return 2 * lane + 1 - orient


@njit(cache=True)
def path_cost(order, orient, cost):
    total = 0.0
    for t in range(1, order.shape[0]):
        total += cost[_exit(order[t - 1], orient[t - 1]), _entry(order[t], orient[t])]
    return total


@njit(cache=True)
def nearest_neighbour(cost, start):
    n = cost.shape[0] // 2
    order = np.empty(n, np.int64)
    orient = np.empty(n, np.int64)
    visited = np.zeros(n, np.bool_)
    first = np.argmin(start)
    order[0], orient[0] = first // 2, first % 2
    visited[order[0]] = True
    for t in range(1, n):
        x = _exit(order[t - 1], orient[t - 1])
        best, bl, bo = np.inf, -1, 0
        for lane in range(n):
            if visited[lane]:
                continue
            if bl == -1:
                bl = lane
            for o in range(2):
                c = cost[x, _entry(lane, o)]
                if c < best:
                    best, bl, bo = c, lane, o
        order[t], orient[t] = bl, bo
        visited[bl] = True
    return order, orient


@njit(cache=True)
def two_opt(order, orient, cost, max_passes):
    """Reverse order[i..j], flipping each lane's direction, whenever that is cheaper.

    Requires a symmetric cost matrix: transitions inside the reversed block then keep their
    cost, so each candidate move is priced from its two boundary transitions alone.
    """
    n = order.shape[0]
    for _ in range(max_passes):
        improved = False
        for i in range(n):
            for j in range(i, n):
                old = 0.0
                new = 0.0
                if i > 0:
                    x = _exit(order[i - 1], orient[i - 1])
                    old += cost[x, _entry(order[i], orient[i])]
                    new += cost[x, _entry(order[j], 1 - orient[j])]
                if j < n - 1:
                    e = _entry(order[j + 1], orient[j + 1])
                    old += cost[_exit(order[j], orient[j]), e]
                    new += cost[_exit(order[i], 1 - orient[i]), e]
                if new < old - 1e-7:
                    order[i : j + 1] = order[i : j + 1][::-1].copy()
                    orient[i : j + 1] = 1 - orient[i : j + 1][::-1]
                    improved = True
        if not improved:
            break
