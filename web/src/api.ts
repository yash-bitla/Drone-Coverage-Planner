import type { Geometry, Job, LngLat, ObstacleIn, PlanRequest, PlanResult } from "./types";

async function body<T>(res: Response): Promise<T> {
  if (!res.ok) {
    const payload = (await res.json().catch(() => ({}))) as { detail?: unknown };
    const detail = payload.detail;
    throw new Error(typeof detail === "string" ? detail : detail ? JSON.stringify(detail) : res.statusText);
  }
  return (await res.json()) as T;
}

async function parse<T>(file: File, kind: "area" | "points" | "obstacles"): Promise<T> {
  const form = new FormData();
  form.append("file", file);
  form.append("kind", kind);
  return body<T>(await fetch("/api/parse", { method: "POST", body: form }));
}

export const parseArea = async (file: File): Promise<Geometry> =>
  (await parse<{ geometry: Geometry }>(file, "area")).geometry;

export const parseStations = async (file: File): Promise<LngLat[]> =>
  (await parse<{ points: LngLat[] }>(file, "points")).points;

export const parseObstacles = async (file: File): Promise<ObstacleIn[]> =>
  (await parse<{ obstacles: ObstacleIn[] }>(file, "obstacles")).obstacles;

export async function submitPlan(request: PlanRequest): Promise<string> {
  const res = await fetch("/api/plans", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(request),
  });
  return (await body<{ id: string }>(res)).id;
}

export async function waitForPlan(id: string, intervalMs = 400): Promise<PlanResult> {
  for (;;) {
    const job = await body<Job>(await fetch(`/api/plans/${id}`));
    if (job.status === "done" && job.result) return job.result;
    if (job.status === "failed") throw new Error(job.error ?? "planning failed");
    await new Promise((resolve) => setTimeout(resolve, intervalMs));
  }
}
