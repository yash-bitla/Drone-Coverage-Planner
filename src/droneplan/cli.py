from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence

from droneplan.config import DroneSpec, PlannerConfig, SensorSpec, StationSpec
from droneplan.errors import PlannerError
from droneplan.export import plan_to_geojson
from droneplan.io import load_area, load_obstacles, load_points
from droneplan.planner import Algorithm, plan_area


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="droneplan", description="Plan multi-drone area mapping.")
    p.add_argument("area", help="area to map (.shp, .zip or .geojson)")
    p.add_argument("--stations", required=True, help="charging stations (point features)")
    p.add_argument("--obstacles", help="obstacle polygons, optionally with a height field")
    p.add_argument("--height-field", default="height_m")
    p.add_argument("--drones", type=int, required=True)
    p.add_argument("--range-km", type=float, required=True, help="battery range in km")
    p.add_argument("--speed", type=float, required=True, help="top speed in m/s")
    p.add_argument("--charge-min", type=float, default=30.0, help="full charge time in minutes")
    p.add_argument("--capacity", type=int, default=1)
    p.add_argument("--altitude", type=float, default=100.0, help="flight altitude in m")
    p.add_argument("--fov", type=float, default=60.0, help="camera field of view in degrees")
    p.add_argument("--overlap", type=float, default=0.2)
    p.add_argument("--clearance", type=float, default=20.0, help="obstacle clearance in m")
    p.add_argument("--algorithm", choices=[a.value for a in Algorithm], default=Algorithm.RSS.value)
    p.add_argument("--out", required=True)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        cfg = PlannerConfig(
            drone=DroneSpec(count=args.drones, range_m=args.range_km * 1000, speed_mps=args.speed),
            station=StationSpec(charge_time_s=args.charge_min * 60, capacity=args.capacity),
            sensor=SensorSpec(altitude_m=args.altitude, fov_deg=args.fov, overlap=args.overlap),
            clearance_m=args.clearance,
        )
        obstacles = load_obstacles(args.obstacles, args.height_field) if args.obstacles else []
        plan = plan_area(
            load_area(args.area),
            load_points(args.stations),
            cfg,
            Algorithm(args.algorithm),
            obstacles,
        )
    except PlannerError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    geojson = plan_to_geojson(plan)
    try:
        with open(args.out, "w") as fh:
            json.dump(geojson, fh)
    except OSError as exc:
        print(f"error: cannot write {args.out}: {exc}", file=sys.stderr)
        return 2
    m = geojson["properties"]["metrics"]
    print(
        f"makespan {m['makespan_s'] / 60:.1f} min (lower bound {m['lower_bound_s'] / 60:.1f}, "
        f"gap {m['gap_pct']:.1f}%), {m['n_flights']} flights, "
        f"{m['unmappable_km2']:.3f} km2 unmappable, solved in {m['solve_time_s']:.2f} s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
