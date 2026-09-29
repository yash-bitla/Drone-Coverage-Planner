import type { GeoJsonObject } from "geojson";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { memo, useEffect, useMemo } from "react";
import { CircleMarker, GeoJSON, MapContainer, Polygon, Polyline, TileLayer, useMap, useMapEvents } from "react-leaflet";
import { droneColor } from "../colors";
import type { DroneSnapshot } from "../timeline";
import { isFlight, isObstacle, type Geometry, type LngLat, type ObstacleIn, type PlanResult } from "../types";

const swap = ([lng, lat]: LngLat): [number, number] => [lat, lng];

function FitToArea({ area }: { area: Geometry | null }) {
  const map = useMap();
  useEffect(() => {
    if (area) map.fitBounds(L.geoJSON(area as GeoJsonObject).getBounds(), { padding: [24, 24] });
  }, [area, map]);
  return null;
}

function ClickToAdd({ onClick }: { onClick?: (p: LngLat) => void }) {
  useMapEvents({ click: (e) => onClick?.([e.latlng.lng, e.latlng.lat]) });
  return null;
}

let nextKey = 0;

interface StaticProps {
  area: Geometry | null;
  obstacles: ObstacleIn[];
  stations: LngLat[];
  plan: PlanResult | null;
}

const StaticLayers = memo(function StaticLayers({ area, obstacles, stations, plan }: StaticProps) {
  // Leaflet GeoJSON layers don't update their data prop, so remount when the input object changes.
  const areaKey = useMemo(() => nextKey++, [area]);
  const obstaclesKey = useMemo(() => nextKey++, [obstacles]);
  const flights = useMemo(
    () => (plan?.features.filter(isFlight) ?? []).map((f) => ({ ...f, path: f.geometry.coordinates.map(swap) })),
    [plan],
  );
  const planObstacles = useMemo(
    () => (plan?.features.filter(isObstacle) ?? []).map((o) => o.geometry.coordinates[0].map(swap)),
    [plan],
  );
  const stationCenters = useMemo(() => stations.map(swap), [stations]);
  return (
    <>
      {area && <GeoJSON key={`a${areaKey}`} data={area as GeoJsonObject} style={{ color: "#1d4ed8", weight: 2, fillOpacity: 0.05 }} />}
      {plan
        ? planObstacles.map((positions, i) => (
            <Polygon key={`po${i}`} positions={positions} pathOptions={{ color: "#475569", fillOpacity: 0.45 }} />
          ))
        : obstacles.map((o, i) => (
            <GeoJSON key={`o${obstaclesKey}-${i}`} data={o.geometry as GeoJsonObject} style={{ color: "#475569", fillOpacity: 0.3 }} />
          ))}
      {flights.map((f, i) => (
        <Polyline
          key={`f${i}`}
          positions={f.path}
          pathOptions={{
            color: droneColor(f.properties.drone),
            weight: 2,
            opacity: 0.45,
            dashArray: f.properties.flight_kind === "reposition" ? "4 6" : undefined,
          }}
        />
      ))}
      {stationCenters.map((c, i) => (
        <CircleMarker key={`s${i}`} center={c} radius={7} pathOptions={{ color: "#111827", fillColor: "#fbbf24", fillOpacity: 1 }} />
      ))}
    </>
  );
});

interface Props {
  area: Geometry | null;
  obstacles: ObstacleIn[];
  stations: LngLat[];
  plan: PlanResult | null;
  snapshot: DroneSnapshot[];
  onMapClick?: (p: LngLat) => void;
}

export function MapView({ area, obstacles, stations, plan, snapshot, onMapClick }: Props) {
  return (
    <MapContainer center={[20, 78]} zoom={5} className="map">
      <TileLayer url="https://tile.openstreetmap.org/{z}/{x}/{y}.png" attribution="&copy; OpenStreetMap contributors" />
      <FitToArea area={area} />
      <ClickToAdd onClick={onMapClick} />
      <StaticLayers area={area} obstacles={obstacles} stations={stations} plan={plan} />
      {snapshot.map((d) => (
        <CircleMarker key={`d${d.drone}`} center={swap(d.position)} radius={6} pathOptions={{ color: "#fff", weight: 2, fillColor: droneColor(d.drone), fillOpacity: 1 }} />
      ))}
    </MapContainer>
  );
}
