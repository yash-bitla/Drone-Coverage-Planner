import numpy as np
from numba import njit


@njit(cache=True)
def split_dp(prefix, near, budget):
    """Prins-style split. f[j] is the least total distance covering tour points 0..j-1 and
    pred[j] the first point of the last sortie; sortie i..j costs near[i] + along + near[j].
    """
    m = near.shape[0]
    f = np.full(m + 1, np.inf)
    pred = np.full(m + 1, -1, np.int64)
    f[0] = 0.0
    for j in range(m):
        i = j
        while i >= 0:
            along = prefix[j] - prefix[i]
            if along > budget:
                break
            if f[i] < np.inf:
                cost = near[i] + along + near[j]
                if cost <= budget and f[i] + cost < f[j + 1]:
                    f[j + 1] = f[i] + cost
                    pred[j + 1] = i
            i -= 1
    return f, pred
