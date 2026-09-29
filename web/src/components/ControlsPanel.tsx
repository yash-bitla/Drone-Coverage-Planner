import { useState } from "react";
import { parseArea, parseObstacles, parseStations } from "../api";
import { canRun, type Inputs } from "../inputs";
import type { Algorithm } from "../types";

const ALGORITHMS: Algorithm[] = ["rss", "rss-fixed-angle", "rss-greedy-split", "rss-full-budget", "darp-stc", "darp-boustrophedon"];

type NumericKey = "drones" | "rangeKm" | "speedMps" | "chargeMin" | "capacity" | "altitudeM" | "fovDeg" | "overlap" | "clearanceM";

const FIELDS: [NumericKey, string, number][] = [
  ["drones", "Drones", 1],
  ["rangeKm", "Range (km)", 0.5],
  ["speedMps", "Top speed (m/s)", 1],
  ["chargeMin", "Charge time, empty to full (min)", 5],
  ["capacity", "Pads per station", 1],
  ["altitudeM", "Altitude (m)", 10],
  ["fovDeg", "Camera FOV (°)", 5],
  ["overlap", "Side overlap (0–1)", 0.05],
  ["clearanceM", "Obstacle clearance (m)", 5],
];

interface Props {
  inputs: Inputs;
  onChange: (patch: Partial<Inputs>) => void;
  onRun: () => void;
  busy: boolean;
  addingStations: boolean;
  onToggleAddStations: () => void;
}

const message = (e: unknown) => (e instanceof Error ? e.message : String(e));

export function ControlsPanel({ inputs, onChange, onRun, busy, addingStations, onToggleAddStations }: Props) {
  const [uploadError, setUploadError] = useState<string | null>(null);

  async function upload<T>(input: HTMLInputElement, parse: (f: File) => Promise<T>, apply: (value: T) => void) {
    const file = input.files?.[0];
    input.value = "";
    if (!file) return;
    setUploadError(null);
    try {
      apply(await parse(file));
    } catch (e) {
      setUploadError(message(e));
    }
  }

  return (
    <section className="panel">
      <h2>Inputs</h2>
      <label>
        Area to map (.zip shapefile or .geojson)
        <input type="file" accept=".zip,.geojson,.json,.gpkg" onChange={(e) => upload(e.target, parseArea, (area) => onChange({ area }))} />
      </label>
      <label>
        Obstacles, optional (height field <code>height_m</code>)
        <input type="file" accept=".zip,.geojson,.json,.gpkg" onChange={(e) => upload(e.target, parseObstacles, (obstacles) => onChange({ obstacles }))} />
      </label>
      <label>
        Charging stations from file, optional
        <input type="file" accept=".zip,.geojson,.json,.gpkg" onChange={(e) => upload(e.target, parseStations, (stations) => onChange({ stations }))} />
      </label>
      <div className="row">
        <button type="button" onClick={onToggleAddStations} className={addingStations ? "active" : ""}>
          {addingStations ? "Click the map to add stations" : "Add stations on the map"}
        </button>
        <button type="button" onClick={() => onChange({ stations: [] })} disabled={!inputs.stations.length}>
          Clear ({inputs.stations.length})
        </button>
      </div>
      {uploadError && <p className="error">{uploadError}</p>}

      {FIELDS.map(([key, label, step]) => (
        <label key={key} className="field">
          {label}
          <input type="number" step={step} value={inputs[key]} onChange={(e) => onChange({ [key]: Number(e.target.value) } as Partial<Inputs>)} />
        </label>
      ))}

      <label className="field">
        Algorithm
        <select value={inputs.algorithm} onChange={(e) => onChange({ algorithm: e.target.value as Algorithm })}>
          {ALGORITHMS.map((a) => (
            <option key={a}>{a}</option>
          ))}
        </select>
      </label>
      <label className="check">
        <input type="checkbox" checked={inputs.compare} onChange={(e) => onChange({ compare: e.target.checked })} />
        Also run the v1 baseline (darp-stc)
      </label>
      <button type="button" className="primary" onClick={onRun} disabled={busy || !canRun(inputs)}>
        {busy ? "Planning…" : "Plan"}
      </button>
    </section>
  );
}
