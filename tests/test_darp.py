import numpy as np
from scipy import ndimage

from droneplan.baselines.darp import darp, farthest_point_seeds


def _regions(assign: np.ndarray, n: int) -> tuple[bool, int]:
    counts = [int((assign == i).sum()) for i in range(n)]
    connected = all(ndimage.label(assign == i)[1] == 1 for i in range(n))
    return connected, max(counts) - min(counts)


def test_two_drones_balanced_and_contiguous() -> None:
    connected, imbalance = _regions(darp(np.ones((10, 10), bool), np.array([[0, 0], [9, 9]])), 2)
    assert connected and imbalance <= 2


def test_three_drones_on_rectangle() -> None:
    req = np.ones((15, 20), dtype=bool)
    connected, imbalance = _regions(darp(req, np.array([[0, 0], [14, 0], [7, 19]])), 3)
    assert connected and imbalance <= 6


def test_duplicate_seeds_do_not_crash() -> None:
    assign = darp(np.ones((4, 4), dtype=bool), np.array([[0, 0], [0, 0]]))
    assert set(np.unique(assign)) <= {0, 1}


def test_farthest_point_seeds_spread_out() -> None:
    xy = np.array([[0.0, 0.0], [1.0, 0.0], [10.0, 0.0]])
    assert farthest_point_seeds(xy, np.array([0.0, 0.0]), 2).tolist() == [0, 2]
