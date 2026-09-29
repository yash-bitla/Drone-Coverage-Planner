import math

import numpy as np
import pytest
from shapely import affinity
from shapely.geometry import Polygon, box

from droneplan.errors import InvalidInputError
from droneplan.geometry.frame import Frame, utm_crs_for
from droneplan.geometry.grid import Grid, rasterize
from droneplan.geometry.obstacles import Obstacle, inflate
from droneplan.geometry.sweep import min_width_sweep_angle


def test_utm_zone_for_central_india() -> None:
    assert utm_crs_for(78.0, 20.0).to_epsg() == 32644


def test_frame_round_trip_and_rotation() -> None:
    base = Frame(utm_crs_for(78.0, 20.0))
    pivot = tuple(base.lnglat_to_utm(np.array([[78.0, 20.0]]))[0])
    frame = base.rotated(math.radians(30), pivot)
    ll = np.array([[78.0, 20.0], [78.01, 20.02]])
    np.testing.assert_allclose(frame.to_lnglat(frame.to_plan(ll)), ll, atol=1e-9)
    np.testing.assert_allclose(frame.to_plan(ll[:1]), [pivot], atol=1e-6)


def test_sweep_angle_follows_long_edge() -> None:
    rect = affinity.rotate(box(0, 0, 1000, 100), 30, origin=(0, 0))
    assert min_width_sweep_angle(rect) == pytest.approx(math.radians(30), abs=1e-9)


def test_obstacle_height_rule() -> None:
    footprint = box(0, 0, 10, 10)
    assert Obstacle(footprint).blocks(altitude_m=100, clearance_m=20)
    assert Obstacle(footprint, 90).blocks(altitude_m=100, clearance_m=20)
    assert not Obstacle(footprint, 50).blocks(altitude_m=100, clearance_m=20)


def test_inflate_merges_and_fills_courtyards() -> None:
    merged = inflate([box(0, 0, 10, 10), box(5, 5, 15, 15)], clearance_m=1, simplify_m=0)
    assert len(merged) == 1 and merged[0].area > 175
    ring = Polygon(box(0, 0, 30, 30).exterior.coords, [box(10, 10, 20, 20).exterior.coords])
    assert inflate([ring], clearance_m=0, simplify_m=0)[0].area == pytest.approx(900)


def test_inflate_keeps_full_clearance_after_simplifying() -> None:
    bulge = Polygon([(0, 0), (50, -5), (100, 0), (100, 100), (0, 100)])
    (grown,) = inflate([bulge], clearance_m=1, simplify_m=10)
    assert grown.covers(bulge.buffer(1))


def test_rasterize_partial_cells_holes_and_obstacles() -> None:
    assert rasterize(box(0, 0, 250, 200), 100, max_cells=100).required.all()
    ring = Polygon(box(0, 0, 500, 500).exterior.coords, [box(200, 200, 300, 300).exterior.coords])
    holed = rasterize(ring, 100, max_cells=100)
    assert holed.shape == (5, 5) and not holed.required[2, 2] and holed.n_unmappable == 0
    walled = rasterize(box(0, 0, 500, 500), 100, max_cells=100, obstacles=[box(210, 210, 290, 290)])
    assert walled.blocked[2, 2] and not walled.required[2, 2]
    assert (walled.n_required, walled.n_unmappable) == (24, 1)


def test_rasterize_rejects_too_many_cells() -> None:
    with pytest.raises(InvalidInputError):
        rasterize(box(0, 0, 1000, 1000), 1, max_cells=100)


def test_grid_centres_follow_convention() -> None:
    grid = Grid.from_mask(np.ones((2, 3), dtype=bool), cell_size=100, origin=(0, 200))
    np.testing.assert_allclose(grid.centers(np.array([[0, 0], [1, 2]])), [[50, 150], [250, 50]])
    assert not grid.blocked.any()
