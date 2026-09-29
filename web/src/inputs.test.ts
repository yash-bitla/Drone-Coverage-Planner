import { describe, expect, it } from "vitest";
import { DEFAULT_INPUTS, canRun, toRequest } from "./inputs";

const AREA = { type: "Polygon" as const, coordinates: [[[78, 20], [78.01, 20], [78.01, 20.01], [78, 20]]] };

describe("inputs", () => {
  it("needs an area and a station to run", () => {
    expect(canRun(DEFAULT_INPUTS)).toBe(false);
    expect(canRun({ ...DEFAULT_INPUTS, area: AREA })).toBe(false);
    expect(canRun({ ...DEFAULT_INPUTS, area: AREA, stations: [[78, 20]] })).toBe(true);
  });

  it("maps camelCase inputs to the API's snake_case request", () => {
    const req = toRequest({ ...DEFAULT_INPUTS, area: AREA, stations: [[78, 20]] }, "darp-stc");
    expect(req).toMatchObject({ range_km: 10, speed_mps: 15, charge_min: 30, algorithm: "darp-stc" });
  });
});
