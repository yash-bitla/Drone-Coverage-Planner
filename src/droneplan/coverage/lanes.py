from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from droneplan._types import BoolArray


@dataclass(frozen=True, slots=True)
class Lane:
    row: int
    col_start: int
    col_end: int


def extract_lanes(mask: BoolArray) -> list[Lane]:
    lanes: list[Lane] = []
    for row in np.flatnonzero(mask.any(axis=1)):
        edges = np.diff(np.concatenate([[0], mask[row].astype(np.int8), [0]]))
        starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1
        lanes.extend(Lane(int(row), int(s), int(e)) for s, e in zip(starts, ends, strict=True))
    return lanes
