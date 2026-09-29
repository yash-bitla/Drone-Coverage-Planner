import numpy as np
import pytest
from shapely.geometry import box

from droneplan.config import DroneSpec, PlannerConfig, StationSpec
from droneplan.errors import InfeasiblePlanError
from droneplan.geometry.routing import Router
from droneplan.scheduling.charging import ChargingNetwork
from droneplan.scheduling.fixed_scheduler import PlannedFlight, fixed_schedule
from droneplan.scheduling.list_scheduler import list_schedule
from droneplan.scheduling.model import Body, FlightKind, initial_stations


def cfg(drones: int, capacity: int = 1) -> PlannerConfig:
    return PlannerConfig(
        drone=DroneSpec(count=drones, range_m=1000, speed_mps=10, accel_mps2=2, reserve_frac=0),
        station=StationSpec(charge_time_s=100, capacity=capacity),
    )


def body(*pts: tuple[float, float]) -> Body:
    xy = np.array(pts, dtype=np.float64)
    return Body(xy=xy, rc=np.zeros((len(xy), 2), dtype=np.int64))


B1, B2 = body((100, 0), (200, 0)), body((0, 100), (0, 200))
ORIGIN = np.array([[0.0, 0.0]])


def test_charging_first_fit() -> None:
    net = ChargingNetwork(n_stations=1, capacity=1)
    net.reserve(0, 0, 10.0, 10.0)
    assert net.earliest_slot(0, 0.0, 5.0) == (0.0, 0)
    assert net.earliest_slot(0, 0.0, 15.0) == (20.0, 0)
    assert net.earliest_slot(0, 12.0, 1.0) == (20.0, 0)
    two = ChargingNetwork(n_stations=1, capacity=2)
    two.reserve(0, 0, 0.0, 100.0)
    assert two.earliest_slot(0, 5.0, 10.0) == (5.0, 1)


def test_single_drone_flies_sequentially() -> None:
    first, second = list_schedule([B1, B2], ORIGIN, cfg(1)).flights
    assert first.t_land == pytest.approx(45.0)
    assert first.charge_end == pytest.approx(85.0)
    assert second.t_launch == pytest.approx(85.0)
    assert second.t_land == pytest.approx(130.0)


def test_two_drones_share_one_pad() -> None:
    s = list_schedule([B1, B2], ORIGIN, cfg(2))
    assert sorted(f.t_launch for f in s.flights) == [0.0, 0.0]
    assert sorted(f.charge_start for f in s.flights) == pytest.approx([45.0, 85.0])
    assert s.makespan_s == pytest.approx(45.0)


def test_repositions_when_nothing_reachable() -> None:
    stations = np.array([[0.0, 0.0], [600.0, 0.0]])
    s = list_schedule([body((1000, 0), (1100, 0))], stations, cfg(1))
    assert [f.kind for f in s.flights] == [FlightKind.REPOSITION, FlightKind.COVERAGE]
    assert s.flights[1].launch_station == 1


def test_unreachable_body_raises() -> None:
    with pytest.raises(InfeasiblePlanError):
        list_schedule([body((600, 0), (700, 0))], ORIGIN, cfg(1))


def test_flights_route_around_obstacles() -> None:
    router = Router([box(40, -10, 60, 10)])
    (flight,) = list_schedule([body((100, 0), (110, 0))], ORIGIN, cfg(1), router).flights
    assert len(flight.path) > 4
    assert router.clear(flight.path[:-1], flight.path[1:]).all()


def test_fixed_schedule_respects_order() -> None:
    path = np.array([[0.0, 0.0], [100.0, 0.0], [0.0, 0.0]])
    pf = PlannedFlight(path, np.zeros((0, 2), np.int64), 0, 0)
    s = fixed_schedule([[pf, pf], [pf]], n_stations=1, cfg=cfg(2))
    d0 = s.by_drone()[0]
    assert d0[1].t_launch == pytest.approx(d0[0].charge_end)
    assert len(s.flights) == 3


def test_initial_stations_round_robin() -> None:
    assert initial_stations(5, 2) == [0, 1, 0, 1, 0]
