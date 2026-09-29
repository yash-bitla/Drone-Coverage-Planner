from __future__ import annotations

import multiprocessing
import os
import shutil
import tempfile
from collections.abc import AsyncIterator
from concurrent.futures import Executor, ProcessPoolExecutor
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Annotated, Any, Literal

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.staticfiles import StaticFiles
from shapely.geometry import mapping

from droneplan import __version__
from droneplan.api.jobs import JobStore
from droneplan.api.schemas import JobOut, PlanRequest
from droneplan.errors import InvalidInputError
from droneplan.io import load_area, load_obstacles, load_points

_UPLOAD_SUFFIXES = {".zip", ".geojson", ".json", ".gpkg"}


def _parse(path: Path, kind: str) -> dict[str, Any]:
    if kind == "area":
        return {"type": "Feature", "geometry": mapping(load_area(path)), "properties": {}}
    if kind == "points":
        return {"points": load_points(path).tolist()}
    return {
        "obstacles": [
            {"geometry": mapping(o.geometry), "height_m": o.height_m} for o in load_obstacles(path)
        ]
    }


def create_app(executor: Executor | None = None, static_dir: Path | None = None) -> FastAPI:
    pool = executor or ProcessPoolExecutor(
        max_workers=int(os.environ.get("DRONEPLAN_WORKERS", "2")),
        mp_context=multiprocessing.get_context("forkserver"),
    )
    store = JobStore(pool)

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        yield
        pool.shutdown(wait=False, cancel_futures=True)

    app = FastAPI(title="droneplan", version=__version__, lifespan=lifespan)

    @app.get("/api/health")
    def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.post("/api/parse")
    def parse(
        file: Annotated[UploadFile, File()],
        kind: Annotated[Literal["area", "points", "obstacles"], Form()],
    ) -> dict[str, Any]:
        name = Path(file.filename or "upload").name
        suffix = Path(name).suffix.lower()
        if suffix not in _UPLOAD_SUFFIXES:
            raise HTTPException(400, f"upload a {', '.join(sorted(_UPLOAD_SUFFIXES))} file")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / f"upload{suffix}"
            with path.open("wb") as fh:
                shutil.copyfileobj(file.file, fh)
            try:
                return _parse(path, kind)
            except InvalidInputError as exc:
                raise HTTPException(400, str(exc).replace(str(path), name)) from exc

    @app.post("/api/plans", status_code=202)
    def submit(req: PlanRequest) -> dict[str, str]:
        try:
            req.area_geometry()
            req.obstacle_list()
        except InvalidInputError as exc:
            raise HTTPException(400, str(exc)) from exc
        return {"id": store.submit(req)}

    @app.get("/api/plans/{job_id}")
    def status(job_id: str) -> JobOut:
        job = store.get(job_id)
        if job is None:
            raise HTTPException(404, "unknown plan id")
        return job

    if static_dir is not None and static_dir.is_dir():
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="web")
    return app


app = create_app(static_dir=Path(os.environ.get("DRONEPLAN_STATIC", "web/dist")))
