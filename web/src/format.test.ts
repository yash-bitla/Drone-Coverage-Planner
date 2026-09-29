import { describe, expect, it } from "vitest";
import { hms, km, pct } from "./format";

describe("format", () => {
  it("formats durations as h:mm:ss", () => {
    expect(hms(0)).toBe("0:00:00");
    expect(hms(3725.4)).toBe("1:02:05");
  });
  it("formats distance and percentages", () => {
    expect(km(12345)).toBe("12.3 km");
    expect(pct(40.777)).toBe("40.8%");
  });
});
