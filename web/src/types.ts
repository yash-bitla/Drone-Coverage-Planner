export type LngLat = [number, number];

export interface Geometry {
  type: "Polygon" | "MultiPolygon";
  coordinates: unknown;
}

export interface ObstacleIn {
  geometry: Geometry;
  height_m: number | null;
}

export type Algorithm =
  | "rss"
  | "rss-fixed-angle"
  | "rss-greedy-split"
  | "rss-full-budget"
  | "darp-stc"
  | "darp-boustrophedon";

export interface StationFeature {
  type: "Feature";
  geometry: { type: "Point"; coordinates: LngLat };
  properties: { kind: "station"; station: number };
}

export interface ObstacleFeature {
  type: "Feature";
  geometry: { type: "Polygon"; coordinates: LngLat[][] };
  properties: { kind: "obstacle" };
}

export interface FlightProperties {
  kind: "flight";
  flight_kind: "coverage" | "reposition";
  drone: number;
  launch_station: number;
  land_station: number;
  t_launch: number;
  t_land: number;
  charge_start: number;
  charge_end: number;
  distance_m: number;
  timestamps: number[];
}

export interface FlightFeature {
  type: "Feature";
  geometry: { type: "LineString"; coordinates: LngLat[] };
  properties: FlightProperties;
}

export type PlanFeature = StationFeature | ObstacleFeature | FlightFeature;

export interface Metrics {
  algorithm: string;
  makespan_s: number;
  lower_bound_s: number;
  gap_pct: number;
  total_distance_m: number;
  n_flights: number;
  n_turns: number;
  coverage_pct: number;
  redundancy_pct: number;
  unmappable_km2: number;
  queue_wait_s: number;
  utilization_pct: number;
  solve_time_s: number;
}

export interface PlanConfig {
  drones: number;
  range_m: number;
  usable_range_m: number;
  speed_mps: number;
  charge_time_s: number;
  capacity: number;
}

export interface PlanResult {
  type: "FeatureCollection";
  features: PlanFeature[];
  properties: { algorithm: Algorithm; metrics: Metrics; config: PlanConfig };
}

export interface PlanRequest {
  area: Geometry;
  stations: LngLat[];
  obstacles: ObstacleIn[];
  drones: number;
  range_km: number;
  speed_mps: number;
  charge_min: number;
  capacity: number;
  altitude_m: number;
  fov_deg: number;
  overlap: number;
  clearance_m: number;
  algorithm: Algorithm;
}

export interface Job {
  id: string;
  status: "pending" | "done" | "failed";
  result: PlanResult | null;
  error: string | null;
}

export const isFlight = (f: PlanFeature): f is FlightFeature => f.properties.kind === "flight";
export const isStation = (f: PlanFeature): f is StationFeature => f.properties.kind === "station";
export const isObstacle = (f: PlanFeature): f is ObstacleFeature => f.properties.kind === "obstacle";
