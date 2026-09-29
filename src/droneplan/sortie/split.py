from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from droneplan._types import FloatArray
from droneplan.errors import InfeasiblePlanError
from droneplan.geometry.routing import Router
from droneplan.kinematics import segment_lengths
from droneplan.sortie import _kernels

_EPS = 1e-9


@dataclass(frozen=True, slots=True)
class Span:
    start: int
    end: int


@dataclass(frozen=True, eq=False)
class SplitInput:
    prefix: FloatArray
    near: FloatArray

    @classmethod
    def from_tour(
        cls, tour_xy: FloatArray, stations_xy: FloatArray, router: Router | None = None
    ) -> SplitInput:
        steps = segment_lengths(tour_xy)
        near, _ = (router or Router()).nearest(tour_xy, stations_xy)
        return cls(prefix=np.concatenate([[0.0], np.cumsum(steps)]), near=near)

    def sortie_distance(self, span: Span) -> float:
        along = self.prefix[span.end] - self.prefix[span.start]
        return float(self.near[span.start] + along + self.near[span.end])

    def check_reachable(self, budget_m: float) -> None:
        unreachable = int((2 * self.near > budget_m + _EPS).sum())
        if unreachable:
            raise InfeasiblePlanError(
                f"{unreachable} tour points are beyond half the sortie budget from every station"
            )


def split_tour(inp: SplitInput, budget_m: float) -> list[Span]:
    if len(inp.near) == 0:
        return []
    inp.check_reachable(budget_m)
    _, pred = _kernels.split_dp(inp.prefix, inp.near, float(budget_m) + _EPS)
    spans: list[Span] = []
    j = len(inp.near)
    while j > 0:
        i = int(pred[j])
        spans.append(Span(i, j - 1))
        j = i
    return spans[::-1]


def greedy_split(inp: SplitInput, budget_m: float) -> list[Span]:
    if len(inp.near) == 0:
        return []
    inp.check_reachable(budget_m)
    spans: list[Span] = []
    start, m = 0, len(inp.near)
    while start < m:
        end = start
        while end + 1 < m and inp.sortie_distance(Span(start, end + 1)) <= budget_m + _EPS:
            end += 1
        spans.append(Span(start, end))
        start = end + 1
    return spans
