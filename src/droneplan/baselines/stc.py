"""Spanning Tree Coverage (Gabriely & Rimon, 2001).

Cells are grouped into 2x2 mega-cells; each mega-cell's four sub-cells form a 4-cycle, and
re-wiring the cycles across every spanning-tree edge yields one Hamiltonian circuit per
connected component. Mega-cells with a blocked, out-of-region, or otherwise ineligible
sub-cell, or on an odd trailing row/column, are left out; the caller covers any cells they
leave behind.
"""

from __future__ import annotations

from collections import defaultdict, deque

import numpy as np
from scipy import ndimage

from droneplan._types import BoolArray, IntArray

Cell = tuple[int, int]


def _subcells(r: int, c: int) -> tuple[Cell, Cell, Cell, Cell]:
    return (2 * r, 2 * c), (2 * r, 2 * c + 1), (2 * r + 1, 2 * c + 1), (2 * r + 1, 2 * c)


def _spanning_tree(mask: BoolArray, root: Cell) -> list[tuple[Cell, Cell]]:
    seen, edges, queue = {root}, [], deque([root])
    while queue:
        r, c = queue.popleft()
        for nb in ((r, c + 1), (r + 1, c), (r, c - 1), (r - 1, c)):
            inside = 0 <= nb[0] < mask.shape[0] and 0 <= nb[1] < mask.shape[1]
            if inside and mask[nb] and nb not in seen:
                seen.add(nb)
                edges.append(((r, c), nb))
                queue.append(nb)
    return edges


def _circuit(mask: BoolArray, root: Cell) -> list[Cell]:
    adj: dict[Cell, set[Cell]] = defaultdict(set)

    def link(a: Cell, b: Cell) -> None:
        adj[a].add(b)
        adj[b].add(a)

    def unlink(a: Cell, b: Cell) -> None:
        adj[a].discard(b)
        adj[b].discard(a)

    for r, c in np.argwhere(mask).tolist():
        tl, tr, br, bl = _subcells(r, c)
        for a, b in ((tl, tr), (tr, br), (br, bl), (bl, tl)):
            link(a, b)
    for a, b in _spanning_tree(mask, root):
        (r1, c1), (r2, c2) = sorted((a, b))
        _a_tl, a_tr, a_br, a_bl = _subcells(r1, c1)
        b_tl, b_tr, _b_br, b_bl = _subcells(r2, c2)
        if r1 == r2:
            unlink(a_tr, a_br)
            unlink(b_tl, b_bl)
            link(a_tr, b_tl)
            link(a_br, b_bl)
        else:
            unlink(a_bl, a_br)
            unlink(b_tl, b_tr)
            link(a_bl, b_tl)
            link(a_br, b_tr)

    start = _subcells(*root)[0]
    circuit, prev, cur = [start], None, start
    while True:
        nxt = next(n for n in sorted(adj[cur]) if n != prev)
        if nxt == start:
            return circuit
        circuit.append(nxt)
        prev, cur = cur, nxt


def stc_path(region: BoolArray, start_rc: tuple[int, int], allowed: BoolArray) -> IntArray:
    h2, w2 = region.shape[0] // 2, region.shape[1] // 2
    quads = (h2, 2, w2, 2)
    mega = region[: 2 * h2, : 2 * w2].reshape(quads).all(axis=(1, 3)) & allowed[
        : 2 * h2, : 2 * w2
    ].reshape(quads).all(axis=(1, 3))
    labels, n_comp = ndimage.label(mega)
    cursor = np.asarray(start_rc, dtype=np.float64)
    remaining = set(range(1, n_comp + 1))
    pieces: list[IntArray] = []
    while remaining:
        options = []
        for comp in remaining:
            megas = np.argwhere(labels == comp)
            d = np.hypot(*(megas * 2 + 0.5 - cursor).T)
            i = int(np.argmin(d))
            options.append((float(d[i]), comp, (int(megas[i, 0]), int(megas[i, 1]))))
        _, comp, root = min(options)
        remaining.discard(comp)
        circuit = np.array(_circuit(labels == comp, root), dtype=np.int64)
        circuit = np.roll(circuit, -int(np.argmin(np.hypot(*(circuit - cursor).T))), axis=0)
        pieces.append(circuit)
        cursor = circuit[-1].astype(np.float64)
    return np.concatenate(pieces) if pieces else np.empty((0, 2), dtype=np.int64)
