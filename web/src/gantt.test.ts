import { describe, expect, it } from "vitest";
import { ganttBars } from "./gantt";
import type { PlanResult } from "./types";

const plan = {
  type: "FeatureCollection",
  features: [
    {
      type: "Feature",
      geometry: { type: "LineString", coordinates: [[0, 0], [1, 0]] },
      properties: {
        kind: "flight", flight_kind: "coverage", drone: 1, launch_station: 0, land_station: 0,
        t_launch: 0, t_land: 40, charge_start: 50, charge_end: 90, distance_m: 400, timestamps: [0, 40],
      },
    },
  ],
  properties: { algorithm: "rss", metrics: {}, config: { drones: 2 } },
} as unknown as PlanResult;

describe("gantt", () => {
  it("emits flight, queue and charge bars per flight", () => {
    const { bars, drones, end } = ganttBars(plan);
    expect(drones).toBe(2);
    expect(end).toBe(90);
    expect(bars).toEqual([
      { drone: 1, start: 0, end: 40, kind: "coverage" },
      { drone: 1, start: 40, end: 50, kind: "queue" },
      { drone: 1, start: 50, end: 90, kind: "charge" },
    ]);
  });

  it("emits reposition bar with no queue when charge_start == t_land", () => {
    const planNoQueue = {
      type: "FeatureCollection",
      features: [
        {
          type: "Feature",
          geometry: { type: "LineString", coordinates: [[0, 0], [1, 0]] },
          properties: {
            kind: "flight", flight_kind: "reposition", drone: 0, launch_station: 0, land_station: 0,
            t_launch: 0, t_land: 40, charge_start: 40, charge_end: 90, distance_m: 400, timestamps: [0, 40],
          },
        },
      ],
      properties: { algorithm: "rss", metrics: {}, config: { drones: 2 } },
    } as unknown as PlanResult;
    const { bars, drones, end } = ganttBars(planNoQueue);
    expect(drones).toBe(2);
    expect(end).toBe(90);
    expect(bars).toEqual([
      { drone: 0, start: 0, end: 40, kind: "reposition" },
      { drone: 0, start: 40, end: 90, kind: "charge" },
    ]);
  });
});
