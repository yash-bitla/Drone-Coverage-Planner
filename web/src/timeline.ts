import { isFlight, isStation, type FlightFeature, type LngLat, type PlanResult } from "./types";

export type DroneState = "idle" | "flying" | "queued" | "charging";

export interface DroneSnapshot {
  drone: number;
  position: LngLat;
  state: DroneState;
  battery: number;
}

interface TimedFlight {
  feature: FlightFeature;
  cumulative: number[];
}

export interface Timeline {
  drones: number;
  end: number;
  stations: LngLat[];
  rangeM: number;
  flightsByDrone: TimedFlight[][];
}

function cumulativeDistance(feature: FlightFeature): number[] {
  const coords = feature.geometry.coordinates;
  const out = [0];
  for (let i = 1; i < coords.length; i++) {
    const [x0, y0] = coords[i - 1];
    const [x1, y1] = coords[i];
    const cosLat = Math.cos(((y0 + y1) / 2) * (Math.PI / 180));
    out.push(out[i - 1] + Math.hypot((x1 - x0) * cosLat, y1 - y0));
  }
  // Rescale to the backend's exact distance so battery at landing matches its charge model.
  const total = out[out.length - 1];
  const scale = total > 0 ? feature.properties.distance_m / total : 0;
  return out.map((d) => d * scale);
}

export function buildTimeline(plan: PlanResult): Timeline {
  const { config } = plan.properties;
  const stations: LngLat[] = [];
  for (const f of plan.features.filter(isStation)) stations[f.properties.station] = f.geometry.coordinates;
  const flightsByDrone: TimedFlight[][] = Array.from({ length: config.drones }, () => []);
  let end = 0;
  for (const f of plan.features.filter(isFlight)) {
    flightsByDrone[f.properties.drone].push({ feature: f, cumulative: cumulativeDistance(f) });
    end = Math.max(end, f.properties.t_land, f.properties.charge_end);
  }
  for (const list of flightsByDrone) list.sort((a, b) => a.feature.properties.t_launch - b.feature.properties.t_launch);
  return { drones: config.drones, end, stations, rangeM: config.range_m, flightsByDrone };
}

function segmentAt(times: number[], t: number): [number, number] {
  let lo = 0;
  let hi = times.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (times[mid] <= t) lo = mid;
    else hi = mid;
  }
  const span = times[hi] - times[lo];
  return [lo, span > 0 ? Math.min(1, Math.max(0, (t - times[lo]) / span)) : 0];
}

function flying(drone: number, { feature, cumulative }: TimedFlight, t: number, rangeM: number): DroneSnapshot {
  const coords = feature.geometry.coordinates;
  const [i, frac] = segmentAt(feature.properties.timestamps, t);
  const j = Math.min(i + 1, coords.length - 1);
  const position: LngLat = [
    coords[i][0] + (coords[j][0] - coords[i][0]) * frac,
    coords[i][1] + (coords[j][1] - coords[i][1]) * frac,
  ];
  const flown = cumulative[i] + (cumulative[j] - cumulative[i]) * frac;
  return { drone, position, state: "flying", battery: Math.max(0, 1 - flown / rangeM) };
}

function grounded(drone: number, last: TimedFlight, t: number, tl: Timeline): DroneSnapshot {
  const p = last.feature.properties;
  const position = tl.stations[p.land_station];
  const landed = Math.max(0, 1 - p.distance_m / tl.rangeM);
  if (t < p.charge_start) return { drone, position, state: "queued", battery: landed };
  if (t < p.charge_end) {
    const frac = (t - p.charge_start) / (p.charge_end - p.charge_start);
    return { drone, position, state: "charging", battery: landed + (1 - landed) * frac };
  }
  return { drone, position, state: "idle", battery: 1 };
}

export function snapshotAt(tl: Timeline, t: number): DroneSnapshot[] {
  return tl.flightsByDrone.map((flights, drone) => {
    const home = tl.stations[drone % Math.max(1, tl.stations.length)];
    if (flights.length === 0 || t < flights[0].feature.properties.t_launch) {
      const start = flights[0] ? tl.stations[flights[0].feature.properties.launch_station] : home;
      return { drone, position: start, state: "idle", battery: 1 };
    }
    let last = flights[0];
    for (const f of flights) {
      const p = f.feature.properties;
      if (t < p.t_launch) break;
      if (t <= p.t_land) return flying(drone, f, t, tl.rangeM);
      last = f;
    }
    return grounded(drone, last, t, tl);
  });
}
