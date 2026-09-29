"""DARP (Kapoutsis et al., 2017), re-implemented as the v1 baseline.

Differences from v1: counts keep a slot for drones that end up with no cells; continuity is
4-connected; multiplicative updates are clamped so energies stay positive; the balance step is
per-drone and halved when that drone's surplus changes sign; the best division seen is returned
rather than the last.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from droneplan._types import BoolArray, FloatArray, IntArray


def farthest_point_seeds(cells_xy: FloatArray, anchor_xy: FloatArray, n: int) -> IntArray:
    def dist_to(i: int) -> FloatArray:
        return np.hypot(*(cells_xy - cells_xy[i]).T)

    chosen = [int(np.argmin(np.hypot(*(cells_xy - anchor_xy).T)))]
    min_d = dist_to(chosen[0])
    for _ in range(n - 1):
        chosen.append(int(np.argmax(min_d)))
        min_d = np.minimum(min_d, dist_to(chosen[-1]))
    return np.array(chosen, dtype=np.int64)


def _continuity_factor(
    region: BoolArray, seed_rc: IntArray, cells: IntArray, step: float
) -> tuple[FloatArray | None, int]:
    labels, n_comp = ndimage.label(region)
    if n_comp <= 1:
        return None, 0
    main = int(labels[tuple(seed_rc)]) or int(np.bincount(labels.ravel())[1:].argmax()) + 1
    to_main = ndimage.distance_transform_edt(labels != main)
    to_other = ndimage.distance_transform_edt(~((labels > 0) & (labels != main)))
    ci = (to_main - to_other)[cells[:, 0], cells[:, 1]]
    return np.clip(1.0 + step * ci, 0.5, 1.5), n_comp - 1


def darp(
    required: BoolArray, seeds_rc: IntArray, *, max_iter: int = 500, patience: int = 100
) -> IntArray:
    cells = np.argwhere(required)
    n = len(seeds_rc)
    out = np.full(required.shape, -1, dtype=np.int64)
    if n == 1:
        out[required] = 0
        return out

    seeds = np.asarray(seeds_rc, dtype=np.int64)
    diff = cells[None, :, :].astype(np.float64) - seeds[:, None, :]
    energy = np.hypot(diff[..., 0], diff[..., 1]) + 1.0
    fair = len(cells) / n
    step = 10.0 ** -np.ceil(np.log10(len(cells)))
    # Per-drone balance step, halved on a sign flip (a fixed step oscillates a boundary back and
    # forth between two drones instead of settling on it). The continuity repair below keeps the
    # fixed `step`: decaying it the same way weakens connectivity repair until it stops working.
    steps = np.full(n, step)
    prev_sign = np.zeros(n)

    best_assign, best_key, stale = None, (np.inf, np.inf), 0
    for _ in range(max_iter):
        assign = np.argmin(energy, axis=0)
        counts = np.bincount(assign, minlength=n)
        split_parts = 0
        for i in range(n):
            region = np.zeros(required.shape, dtype=bool)
            mine = cells[assign == i]
            region[mine[:, 0], mine[:, 1]] = True
            factor, extra = _continuity_factor(region, seeds[i], cells, step)
            split_parts += extra
            if factor is not None:
                energy[i] *= factor
        key = (split_parts, int(counts.max() - counts.min()))
        if key < best_key:
            best_assign, best_key, stale = assign.copy(), key, 0
        else:
            stale += 1
        if (key[0] == 0 and key[1] <= 1) or stale >= patience:
            break
        cur_sign = np.sign(counts - fair)
        flipped = (cur_sign != 0) & (prev_sign != 0) & (cur_sign != prev_sign)
        steps[flipped] *= 0.5
        prev_sign = np.where(cur_sign != 0, cur_sign, prev_sign)
        energy *= np.clip(1.0 + steps * (counts - fair), 0.5, 1.5)[:, None]

    assert best_assign is not None
    out[cells[:, 0], cells[:, 1]] = best_assign
    return out
