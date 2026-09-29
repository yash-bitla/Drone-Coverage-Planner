import type { GeoJsonObject } from "geojson";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { useEffect } from "react";
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

interface Props {
  area: Geometry | null;
  obstacles: ObstacleIn[];
  stations: LngLat[];
  plan: PlanResult | null;
  snapshot: DroneSnapshot[];
  onMapClick?: (p: LngLat) => void;
}

export function MapView({ area, obstacles, stations, plan, snapshot, onMapClick }: Props) {
  const flights = plan?.features.filter(isFlight) ?? [];
  const planObstacles = plan?.features.filter(isObstacle) ?? [];
  return (
    <MapContainer center={[20, 78]} zoom={5} className="map">
      <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" attribution="&copy; OpenStreetMap contributors" />
      <FitToArea area={area} />
      <ClickToAdd onClick={onMapClick} />
      {area && <GeoJSON key={JSON.stringify(area)} data={area as GeoJsonObject} style={{ color: "#1d4ed8", weight: 2, fillOpacity: 0.05 }} />}
      {plan
        ? planObstacles.map((o, i) => (
            <Polygon key={`po${i}`} positions={o.geometry.coordinates[0].map(swap)} pathOptions={{ color: "#475569", fillOpacity: 0.45 }} />
          ))
        : obstacles.map((o, i) => (
            <GeoJSON key={`o${i}${JSON.stringify(o.geometry).length}`} data={o.geometry as GeoJsonObject} style={{ color: "#475569", fillOpacity: 0.3 }} />
          ))}
      {flights.map((f, i) => (
        <Polyline
          key={`f${i}`}
          positions={f.geometry.coordinates.map(swap)}
          pathOptions={{
            color: droneColor(f.properties.drone),
            weight: 2,
            opacity: 0.45,
            dashArray: f.properties.flight_kind === "reposition" ? "4 6" : undefined,
          }}
        />
      ))}
      {stations.map((s, i) => (
        <CircleMarker key={`s${i}`} center={swap(s)} radius={7} pathOptions={{ color: "#111827", fillColor: "#fbbf24", fillOpacity: 1 }} />
      ))}
      {snapshot.map((d) => (
        <CircleMarker key={`d${d.drone}`} center={swap(d.position)} radius={6} pathOptions={{ color: "#fff", weight: 2, fillColor: droneColor(d.drone), fillOpacity: 1 }} />
      ))}
    </MapContainer>
  );
}
