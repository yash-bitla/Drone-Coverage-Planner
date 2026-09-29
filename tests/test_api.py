import io
import tempfile
import threading
import time
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import geopandas as gpd
import pytest
from fastapi.testclient import TestClient
from shapely.geometry import Point, box

from droneplan.api import jobs
from droneplan.api.app import create_app
from droneplan.api.jobs import JobStore
from droneplan.api.schemas import PlanRequest

AREA = {
    "type": "Polygon",
    "coordinates": [[[78.0, 20.0], [78.01, 20.0], [78.01, 20.008], [78.0, 20.008], [78.0, 20.0]]],
}
BODY = {"area": AREA, "stations": [[78.0, 20.0]], "drones": 2, "range_km": 8, "speed_mps": 12}


def client() -> TestClient:
    return TestClient(create_app(executor=ThreadPoolExecutor(max_workers=1)))


def wait(c: TestClient, job_id: str) -> dict[str, Any]:
    for _ in range(600):
        body = c.get(f"/api/plans/{job_id}").json()
        if body["status"] != "pending":
            return body
        time.sleep(0.1)
    raise AssertionError("job did not finish")


def test_health() -> None:
    assert client().get("/api/health").json() == {"status": "ok"}


def test_plan_round_trip() -> None:
    c = client()
    res = c.post("/api/plans", json=BODY)
    assert res.status_code == 202
    job = wait(c, res.json()["id"])
    assert job["status"] == "done"
    assert job["result"]["type"] == "FeatureCollection"
    assert job["result"]["properties"]["metrics"]["coverage_pct"] == 100.0


def test_infeasible_plan_reports_failure() -> None:
    c = client()
    job = wait(c, c.post("/api/plans", json={**BODY, "range_km": 0.05}).json()["id"])
    assert job["status"] == "failed"
    assert job["error"] and job["error"] != "internal error"


def test_bad_geometry_is_rejected_synchronously() -> None:
    res = client().post(
        "/api/plans", json={**BODY, "area": {"type": "Point", "coordinates": [78, 20]}}
    )
    assert res.status_code == 400


def test_unknown_job_is_404() -> None:
    assert client().get("/api/plans/nope").status_code == 404


def test_parse_geojson_and_zipped_shapefile(tmp_path: Path) -> None:
    c = client()
    gpd.GeoDataFrame(geometry=[Point(78.0, 20.0)], crs=4326).to_file(tmp_path / "s.geojson")
    res = c.post(
        "/api/parse",
        data={"kind": "points"},
        files={"file": ("s.geojson", (tmp_path / "s.geojson").read_bytes())},
    )
    assert res.json() == {"points": [[78.0, 20.0]]}

    gpd.GeoDataFrame(geometry=[box(78.0, 20.0, 78.01, 20.01)], crs=4326).to_file(
        tmp_path / "area.shp"
    )
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for part in tmp_path.glob("area.*"):
            zf.write(part, part.name)
    res = c.post("/api/parse", data={"kind": "area"}, files={"file": ("area.zip", buf.getvalue())})
    assert res.json()["geometry"]["type"] == "Polygon"


def test_parse_rejects_unknown_suffix() -> None:
    res = client().post("/api/parse", data={"kind": "area"}, files={"file": ("a.txt", b"x")})
    assert res.status_code == 400


def test_parse_obstacles(tmp_path: Path) -> None:
    gpd.GeoDataFrame(
        {"height_m": [30.0]}, geometry=[box(78.0, 20.0, 78.01, 20.01)], crs=4326
    ).to_file(tmp_path / "o.geojson")
    res = client().post(
        "/api/parse",
        data={"kind": "obstacles"},
        files={"file": ("o.geojson", (tmp_path / "o.geojson").read_bytes())},
    )
    obstacles = res.json()["obstacles"]
    assert len(obstacles) == 1
    assert obstacles[0]["geometry"]["type"] == "Polygon"
    assert obstacles[0]["height_m"] == 30.0


def _finish(store: JobStore, job_id: str) -> None:
    for _ in range(600):
        job = store.get(job_id)
        assert job is not None
        if job.status != "pending":
            return
        time.sleep(0.1)
    raise AssertionError("job did not finish")


def test_eviction_drops_oldest_finished_jobs() -> None:
    store = JobStore(ThreadPoolExecutor(max_workers=1), max_jobs=2)
    req = PlanRequest.model_validate(BODY)
    ids = []
    for _ in range(3):
        ids.append(store.submit(req))
        _finish(store, ids[-1])
    assert store.get(ids[0]) is None
    assert store.get(ids[1]) is not None
    assert store.get(ids[2]) is not None


def test_eviction_keeps_unfinished_jobs(monkeypatch: pytest.MonkeyPatch) -> None:
    gate = threading.Event()
    monkeypatch.setattr(jobs, "run_plan", lambda _: gate.wait(10) and {})
    store = JobStore(ThreadPoolExecutor(max_workers=1), max_jobs=1)
    req = PlanRequest.model_validate(BODY)
    first = store.submit(req)
    second = store.submit(req)
    assert store.get(first) is not None
    assert store.get(second) is not None
    gate.set()


def test_unexpected_exception_is_reported_as_internal_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def boom(_: dict[str, Any]) -> dict[str, Any]:
        raise RuntimeError("secret detail")

    monkeypatch.setattr(jobs, "run_plan", boom)
    store = JobStore(ThreadPoolExecutor(max_workers=1))
    job_id = store.submit(PlanRequest.model_validate(BODY))
    _finish(store, job_id)
    job = store.get(job_id)
    assert job is not None
    assert job.status == "failed"
    assert job.error == "internal error"


def test_parse_error_names_the_upload_not_the_temp_path() -> None:
    res = client().post(
        "/api/parse", data={"kind": "area"}, files={"file": ("mine.geojson", b"not geojson")}
    )
    assert res.status_code == 400
    detail = res.json()["detail"]
    assert detail.startswith("mine.geojson: ")
    assert tempfile.gettempdir() not in detail
