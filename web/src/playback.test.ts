import { describe, expect, it } from "vitest";
import { clampT, startT } from "./playback";

describe("playback helpers", () => {
  it("clamps t to a shorter end and leaves smaller t alone", () => {
    expect(clampT(5000, 3000)).toBe(3000);
    expect(clampT(1000, 3000)).toBe(1000);
  });
  it("restarts from 0 when starting at or past the end", () => {
    expect(startT(3000, 3000)).toBe(0);
    expect(startT(5000, 3000)).toBe(0);
    expect(startT(1200, 3000)).toBe(1200);
  });
});
