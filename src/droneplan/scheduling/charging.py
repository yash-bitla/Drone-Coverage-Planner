from __future__ import annotations

import bisect


def _first_fit(intervals: list[tuple[float, float]], ready: float, duration: float) -> float:
    t = ready
    for start, end in intervals:
        if end <= t:
            continue
        if t + duration <= start:
            return t
        t = end
    return t


class ChargingNetwork:
    def __init__(self, n_stations: int, capacity: int) -> None:
        self._busy: list[list[list[tuple[float, float]]]] = [
            [[] for _ in range(capacity)] for _ in range(n_stations)
        ]

    def earliest_slot(self, station: int, ready: float, duration: float) -> tuple[float, int]:
        best_start, best_pad = float("inf"), -1
        for pad, intervals in enumerate(self._busy[station]):
            start = _first_fit(intervals, ready, duration)
            if start < best_start:
                best_start, best_pad = start, pad
        return best_start, best_pad

    def reserve(self, station: int, pad: int, start: float, duration: float) -> None:
        if duration > 0:
            bisect.insort(self._busy[station][pad], (start, start + duration))
