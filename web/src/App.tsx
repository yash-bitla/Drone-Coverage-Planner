import { useMemo, useState } from "react";
import { submitPlan, waitForPlan } from "./api";
import { ControlsPanel } from "./components/ControlsPanel";
import { DroneStatus } from "./components/DroneStatus";
import { MapView } from "./components/MapView";
import { MetricsTable } from "./components/MetricsTable";
import { PlaybackBar } from "./components/PlaybackBar";
import { ScheduleGantt } from "./components/ScheduleGantt";
import { DEFAULT_INPUTS, toRequest, type Inputs } from "./inputs";
import { usePlayback } from "./playback";
import { buildTimeline, snapshotAt } from "./timeline";
import type { Algorithm, LngLat, PlanResult } from "./types";

type Slot = "primary" | "baseline";

export function App() {
  const [inputs, setInputs] = useState<Inputs>(DEFAULT_INPUTS);
  const [addingStations, setAddingStations] = useState(false);
  const [results, setResults] = useState<Partial<Record<Slot, PlanResult>>>({});
  const [shown, setShown] = useState<Slot>("primary");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const plan = results[shown] ?? results.primary ?? null;
  const timeline = useMemo(() => (plan ? buildTimeline(plan) : null), [plan]);
  const playback = usePlayback(timeline?.end ?? 0);
  const snapshot = timeline ? snapshotAt(timeline, playback.t) : [];

  const update = (patch: Partial<Inputs>) => setInputs((prev) => ({ ...prev, ...patch }));
  const addStation = (p: LngLat) => addingStations && update({ stations: [...inputs.stations, p] });

  async function run() {
    setBusy(true);
    setError(null);
    setResults({});
    playback.reset();
    try {
      const jobs: [Slot, Algorithm][] = [["primary", inputs.algorithm]];
      if (inputs.compare && inputs.algorithm !== "darp-stc") jobs.push(["baseline", "darp-stc"]);
      const done = await Promise.all(
        jobs.map(async ([slot, algorithm]) => [slot, await waitForPlan(await submitPlan(toRequest(inputs, algorithm)))] as const),
      );
      setResults(Object.fromEntries(done));
      setShown("primary");
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="layout">
      <aside>
        <h1>droneplan</h1>
        <ControlsPanel
          inputs={inputs}
          onChange={update}
          onRun={run}
          busy={busy}
          addingStations={addingStations}
          onToggleAddStations={() => setAddingStations((v) => !v)}
        />
        {error && <p className="error">{error}</p>}
        <MetricsTable primary={results.primary} baseline={results.baseline} />
      </aside>
      <main>
        <MapView area={inputs.area} obstacles={inputs.obstacles} stations={inputs.stations} plan={plan} snapshot={snapshot} onMapClick={addStation} />
        {plan && timeline && (
          <section className="bottom">
            {results.baseline && (
              <div className="row">
                {(["primary", "baseline"] as const).map((slot) => (
                  <button key={slot} type="button" className={shown === slot ? "active" : ""} onClick={() => setShown(slot)}>
                    Show {results[slot]?.properties.algorithm}
                  </button>
                ))}
              </div>
            )}
            <PlaybackBar
              t={playback.t}
              end={timeline.end}
              playing={playback.playing}
              speed={playback.speed}
              onSeek={playback.seek}
              onPlayPause={() => playback.setPlaying(!playback.playing)}
              onSpeed={playback.setSpeed}
            />
            <ScheduleGantt plan={plan} t={playback.t} />
            <DroneStatus snapshot={snapshot} />
          </section>
        )}
      </main>
    </div>
  );
}
