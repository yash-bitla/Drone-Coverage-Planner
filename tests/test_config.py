import pytest

from droneplan.config import DroneSpec, PlannerConfig, SensorSpec, StationSpec
from droneplan.errors import InvalidInputError


def test_footprint_matches_spec_default() -> None:
    assert SensorSpec().footprint_m == pytest.approx(92.376, abs=1e-3)


def test_usable_range_applies_reserve() -> None:
    assert DroneSpec(count=2, range_m=10_000, speed_mps=15).usable_range_m == pytest.approx(9_000)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"count": 0, "range_m": 1.0, "speed_mps": 1.0},
        {"count": 1, "range_m": 0.0, "speed_mps": 1.0},
        {"count": 1, "range_m": 1.0, "speed_mps": 0.0},
        {"count": 1, "range_m": 1.0, "speed_mps": 1.0, "accel_mps2": 0.0},
        {"count": 1, "range_m": 1.0, "speed_mps": 1.0, "reserve_frac": 1.0},
    ],
)
def test_invalid_drone_rejected(kwargs: dict[str, float]) -> None:
    with pytest.raises(InvalidInputError):
        DroneSpec(**kwargs)  # type: ignore[arg-type]


def test_invalid_station_sensor_and_config_rejected() -> None:
    with pytest.raises(InvalidInputError):
        StationSpec(capacity=0)
    with pytest.raises(InvalidInputError):
        StationSpec(charge_time_s=-1)
    with pytest.raises(InvalidInputError):
        SensorSpec(overlap=1.0)
    with pytest.raises(InvalidInputError):
        PlannerConfig(drone=DroneSpec(count=1, range_m=1, speed_mps=1), clearance_m=-1)


def test_planner_config_defaults() -> None:
    cfg = PlannerConfig(drone=DroneSpec(count=1, range_m=5_000, speed_mps=10))
    assert (cfg.station.capacity, cfg.clearance_m, cfg.max_cells) == (1, 20.0, 1_000_000)
