import type { Algorithm, Geometry, LngLat, ObstacleIn, PlanRequest } from "./types";

export interface Inputs {
  area: Geometry | null;
  obstacles: ObstacleIn[];
  stations: LngLat[];
  drones: number;
  rangeKm: number;
  speedMps: number;
  chargeMin: number;
  capacity: number;
  altitudeM: number;
  fovDeg: number;
  overlap: number;
  clearanceM: number;
  algorithm: Algorithm;
  compare: boolean;
}

export const DEFAULT_INPUTS: Inputs = {
  area: null,
  obstacles: [],
  stations: [],
  drones: 4,
  rangeKm: 10,
  speedMps: 15,
  chargeMin: 30,
  capacity: 1,
  altitudeM: 100,
  fovDeg: 60,
  overlap: 0.2,
  clearanceM: 20,
  algorithm: "rss",
  compare: true,
};

export const canRun = (inputs: Inputs): boolean => inputs.area !== null && inputs.stations.length > 0;

export function toRequest(inputs: Inputs, algorithm: Algorithm): PlanRequest {
  if (!inputs.area) throw new Error("an area is required");
  return {
    area: inputs.area,
    stations: inputs.stations,
    obstacles: inputs.obstacles,
    drones: inputs.drones,
    range_km: inputs.rangeKm,
    speed_mps: inputs.speedMps,
    charge_min: inputs.chargeMin,
    capacity: inputs.capacity,
    altitude_m: inputs.altitudeM,
    fov_deg: inputs.fovDeg,
    overlap: inputs.overlap,
    clearance_m: inputs.clearanceM,
    algorithm,
  };
}
