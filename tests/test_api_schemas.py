import pytest
from pydantic import ValidationError

from droneplan.api.schemas import PlanRequest
from droneplan.errors import InvalidInputError
from droneplan.planner import Algorithm

AREA = {
    "type": "Polygon",
    "coordinates": [[[78.0, 20.0], [78.01, 20.0], [78.01, 20.01], [78.0, 20.01], [78.0, 20.0]]],
}


def request(**overrides: object) -> PlanRequest:
    body = {"area": AREA, "stations": [[78.0, 20.0]], "drones": 2, "range_km": 8, "speed_mps": 12}
    return PlanRequest.model_validate({**body, **overrides})


def test_defaults_map_to_planner_config() -> None:
    cfg = request().config()
    assert (cfg.drone.count, cfg.drone.range_m, cfg.drone.speed_mps) == (2, 8000.0, 12.0)
    assert (cfg.station.charge_time_s, cfg.station.capacity, cfg.clearance_m) == (1800.0, 1, 20.0)
    assert request().algorithm is Algorithm.RSS


def test_geometry_and_obstacles_convert() -> None:
    req = request(obstacles=[{"geometry": AREA, "height_m": 40}])
    assert req.area_geometry().geom_type == "Polygon"
    (obstacle,) = req.obstacle_list()
    assert obstacle.height_m == 40


@pytest.mark.parametrize(
    "field,value", [("drones", 0), ("range_km", 0), ("stations", []), ("overlap", 1.0)]
)
def test_rejects_out_of_range_fields(field: str, value: object) -> None:
    with pytest.raises(ValidationError):
        request(**{field: value})


def test_rejects_non_polygon_area() -> None:
    with pytest.raises(InvalidInputError):
        request(area={"type": "Point", "coordinates": [78.0, 20.0]}).area_geometry()
