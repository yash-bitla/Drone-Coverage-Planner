import { droneColor } from "../colors";
import type { DroneSnapshot } from "../timeline";

export function DroneStatus({ snapshot }: { snapshot: DroneSnapshot[] }) {
  return (
    <ul className="drones">
      {snapshot.map((d) => (
        <li key={d.drone}>
          <span className="swatch" style={{ background: droneColor(d.drone) }} />
          <span className="name">Drone {d.drone + 1}</span>
          <span className={`state ${d.state}`}>{d.state}</span>
          <span className="battery">
            <span style={{ width: `${Math.round(d.battery * 100)}%` }} />
          </span>
        </li>
      ))}
    </ul>
  );
}
