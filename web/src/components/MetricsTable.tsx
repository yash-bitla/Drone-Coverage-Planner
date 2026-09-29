import { hms, km, pct } from "../format";
import type { Metrics, PlanResult } from "../types";

const ROWS: [string, (m: Metrics) => string][] = [
  ["Makespan", (m) => hms(m.makespan_s)],
  ["Lower bound", (m) => hms(m.lower_bound_s)],
  ["Gap to bound", (m) => pct(m.gap_pct)],
  ["Flights", (m) => String(m.n_flights)],
  ["Turns", (m) => String(m.n_turns)],
  ["Distance", (m) => km(m.total_distance_m)],
  ["Coverage", (m) => pct(m.coverage_pct)],
  ["Redundancy", (m) => pct(m.redundancy_pct)],
  ["Queue wait", (m) => hms(m.queue_wait_s)],
  ["Utilisation", (m) => pct(m.utilization_pct)],
  ["Unmappable", (m) => `${m.unmappable_km2.toFixed(3)} km²`],
  ["Solve time", (m) => `${m.solve_time_s.toFixed(2)} s`],
];

export function MetricsTable({ primary, baseline }: { primary?: PlanResult; baseline?: PlanResult }) {
  const plans = [primary, baseline].filter((p): p is PlanResult => p !== undefined);
  if (!plans.length) return null;
  const speedup =
    primary && baseline
      ? (1 - primary.properties.metrics.makespan_s / baseline.properties.metrics.makespan_s) * 100
      : null;
  return (
    <section className="panel">
      <h2>Results</h2>
      {speedup !== null && <p className="headline">{pct(speedup)} faster than the v1 baseline</p>}
      <table>
        <thead>
          <tr>
            <th />
            {plans.map((p) => (
              <th key={p.properties.algorithm}>{p.properties.algorithm}</th>
            ))}
          </tr>
        </thead>
        <tbody>
          {ROWS.map(([label, value]) => (
            <tr key={label}>
              <td>{label}</td>
              {plans.map((p) => (
                <td key={p.properties.algorithm}>{value(p.properties.metrics)}</td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    </section>
  );
}
