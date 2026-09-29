import { useMemo } from "react";
import { droneColor } from "../colors";
import { ganttBars } from "../gantt";
import type { PlanResult } from "../types";

const ROW = 18;
const WIDTH = 1000;
const FILL = { queue: "#e5e7eb", charge: "#86efac", reposition: "#9ca3af" } as const;

export function ScheduleGantt({ plan, t }: { plan: PlanResult; t: number }) {
  const { bars, drones, end } = useMemo(() => ganttBars(plan), [plan]);
  const x = (s: number) => (end > 0 ? (s / end) * WIDTH : 0);
  return (
    <svg className="gantt" viewBox={`0 0 ${WIDTH} ${drones * ROW}`} preserveAspectRatio="none">
      {bars.map((b, i) => (
        <rect
          key={i}
          x={x(b.start)}
          y={b.drone * ROW + 2}
          width={Math.max(1, x(b.end) - x(b.start))}
          height={ROW - 4}
          fill={b.kind === "coverage" ? droneColor(b.drone) : FILL[b.kind]}
        >
          <title>{`Drone ${b.drone + 1}: ${b.kind}`}</title>
        </rect>
      ))}
      <line x1={x(t)} x2={x(t)} y1={0} y2={drones * ROW} stroke="#111827" strokeWidth={2} />
    </svg>
  );
}
