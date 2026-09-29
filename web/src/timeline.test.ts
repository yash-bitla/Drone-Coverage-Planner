import { describe, expect, it } from "vitest";
import { buildTimeline, snapshotAt } from "./timeline";
import type { FlightFeature, PlanResult } from "./types";

function flight(t0: number, t1: number, c0: number, c1: number, coords: [number, number][], times: number[]): FlightFeature {
  return {
    type: "Feature",
    geometry: { type: "LineString", coordinates: coords },
    properties: {
      kind: "flight", flight_kind: "coverage", drone: 0, launch_station: 0, land_station: 0,
      t_launch: t0, t_land: t1, charge_start: c0, charge_end: c1, distance_m: 400, timestamps: times,
    },
  };
}

const PLAN: PlanResult = {
  type: "FeatureCollection",
  features: [
    { type: "Feature", geometry: { type: "Point", coordinates: [0, 0] }, properties: { kind: "station", station: 0 } },
    flight(0, 40, 50, 90, [[0, 0], [1, 0], [0, 0]], [0, 20, 40]),
    flight(90, 130, 130, 170, [[0, 0], [0, 1], [0, 0]], [90, 110, 130]),
  ],
  properties: {
    algorithm: "rss",
    metrics: {} as PlanResult["properties"]["metrics"],
    config: { drones: 2, range_m: 1000, usable_range_m: 900, speed_mps: 10, charge_time_s: 100, capacity: 1 },
  },
};

describe("timeline", () => {
  const tl = buildTimeline(PLAN);

  it("ends when the last charge ends", () => {
    expect(tl.end).toBe(170);
  });

  it("interpolates position and drains battery while flying", () => {
    const [d0] = snapshotAt(tl, 10);
    expect(d0.state).toBe("flying");
    expect(d0.position[0]).toBeCloseTo(0.5);
    expect(d0.battery).toBeCloseTo(0.9);
  });

  it("queues, then charges linearly back to full", () => {
    expect(snapshotAt(tl, 45)[0]).toMatchObject({ state: "queued", position: [0, 0] });
    expect(snapshotAt(tl, 45)[0].battery).toBeCloseTo(0.6);
    expect(snapshotAt(tl, 70)[0].state).toBe("charging");
    expect(snapshotAt(tl, 70)[0].battery).toBeCloseTo(0.8);
    expect(snapshotAt(tl, 89)[0]).toMatchObject({ state: "charging" });
  });

  it("idles full between charge end and next launch, and after the plan", () => {
    expect(snapshotAt(tl, 170)[0]).toMatchObject({ state: "idle", battery: 1 });
  });

  it("idles with full battery between charge end and next launch gap", () => {
    const planWithGap: PlanResult = {
      type: "FeatureCollection",
      features: [
        { type: "Feature", geometry: { type: "Point", coordinates: [0, 0] }, properties: { kind: "station", station: 0 } },
        flight(0, 40, 50, 90, [[0, 0], [1, 0], [0, 0]], [0, 20, 40]),
        flight(120, 160, 160, 200, [[0, 0], [0, 1], [0, 0]], [120, 140, 160]),
      ],
      properties: {
        algorithm: "rss",
        metrics: {} as PlanResult["properties"]["metrics"],
        config: { drones: 2, range_m: 1000, usable_range_m: 900, speed_mps: 10, charge_time_s: 100, capacity: 1 },
      },
    };
    const tlGap = buildTimeline(planWithGap);
    expect(snapshotAt(tlGap, 105)[0]).toMatchObject({ state: "idle", battery: 1, position: [0, 0] });
  });

  it("keeps drones without flights idle at their round-robin station", () => {
    const planMultiStation: PlanResult = {
      type: "FeatureCollection",
      features: [
        { type: "Feature", geometry: { type: "Point", coordinates: [0, 0] }, properties: { kind: "station", station: 0 } },
        { type: "Feature", geometry: { type: "Point", coordinates: [1, 1] }, properties: { kind: "station", station: 1 } },
        flight(0, 40, 50, 90, [[0, 0], [1, 0], [0, 0]], [0, 20, 40]),
      ],
      properties: {
        algorithm: "rss",
        metrics: {} as PlanResult["properties"]["metrics"],
        config: { drones: 3, range_m: 1000, usable_range_m: 900, speed_mps: 10, charge_time_s: 100, capacity: 1 },
      },
    };
    const tlMulti = buildTimeline(planMultiStation);
    expect(snapshotAt(tlMulti, 10)[1]).toMatchObject({ drone: 1, state: "idle", battery: 1, position: [1, 1] });
    expect(snapshotAt(tlMulti, 10)[2]).toMatchObject({ drone: 2, state: "idle", battery: 1, position: [0, 0] });
  });

  it("interpolates position and battery during second flight", () => {
    const [d0] = snapshotAt(tl, 100);
    expect(d0.state).toBe("flying");
    expect(d0.position).toEqual([0, 0.5]);
    expect(d0.battery).toBeCloseTo(0.9);
  });
});
