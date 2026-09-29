from __future__ import annotations

import logging
import threading
import uuid
from collections import OrderedDict
from concurrent.futures import Executor, Future
from typing import Any

import numpy as np

from droneplan.api.schemas import JobOut, JobStatus, PlanRequest
from droneplan.errors import PlannerError
from droneplan.export import plan_to_geojson
from droneplan.planner import plan_area

_log = logging.getLogger(__name__)


def run_plan(request_json: dict[str, Any]) -> dict[str, Any]:
    req = PlanRequest.model_validate(request_json)
    plan = plan_area(
        req.area_geometry(),
        np.asarray(req.stations),
        req.config(),
        req.algorithm,
        req.obstacle_list(),
    )
    return plan_to_geojson(plan)


class JobStore:
    """In-memory jobs; the oldest finished jobs are evicted beyond `max_jobs`."""

    def __init__(self, executor: Executor, max_jobs: int = 100) -> None:
        self._executor = executor
        self._max_jobs = max_jobs
        self._jobs: OrderedDict[str, Future[dict[str, Any]]] = OrderedDict()
        self._lock = threading.Lock()

    def submit(self, req: PlanRequest) -> str:
        job_id = uuid.uuid4().hex
        future = self._executor.submit(run_plan, req.model_dump(mode="json"))
        with self._lock:
            self._jobs[job_id] = future
            self._evict()
        return job_id

    def get(self, job_id: str) -> JobOut | None:
        with self._lock:
            future = self._jobs.get(job_id)
        if future is None:
            return None
        if not future.done():
            return JobOut(id=job_id, status=JobStatus.PENDING)
        exc = future.exception()
        if isinstance(exc, PlannerError):
            return JobOut(id=job_id, status=JobStatus.FAILED, error=str(exc))
        if exc is not None:
            _log.error("plan job %s crashed", job_id, exc_info=exc)
            return JobOut(id=job_id, status=JobStatus.FAILED, error="internal error")
        return JobOut(id=job_id, status=JobStatus.DONE, result=future.result())

    def _evict(self) -> None:
        finished = [k for k, f in self._jobs.items() if f.done()]
        for job_id in finished[: max(0, len(self._jobs) - self._max_jobs)]:
            del self._jobs[job_id]
