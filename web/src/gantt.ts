import { isFlight, type PlanResult } from "./types";

export interface GanttBar {
  drone: number;
  start: number;
  end: number;
  kind: "coverage" | "reposition" | "queue" | "charge";
}

export function ganttBars(plan: PlanResult): { bars: GanttBar[]; drones: number; end: number } {
  const bars: GanttBar[] = [];
  let end = 0;
  for (const f of plan.features.filter(isFlight)) {
    const p = f.properties;
    bars.push({ drone: p.drone, start: p.t_launch, end: p.t_land, kind: p.flight_kind });
    if (p.charge_start > p.t_land) bars.push({ drone: p.drone, start: p.t_land, end: p.charge_start, kind: "queue" });
    if (p.charge_end > p.charge_start) bars.push({ drone: p.drone, start: p.charge_start, end: p.charge_end, kind: "charge" });
    end = Math.max(end, p.t_land, p.charge_end);
  }
  return { bars, drones: plan.properties.config.drones, end };
}
