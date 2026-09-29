from __future__ import annotations

import argparse
import os
from collections.abc import Sequence
from concurrent.futures import ProcessPoolExecutor
from itertools import product
from pathlib import Path
from typing import Any

import pandas as pd

from benchmarks.datasets import Instance, synthetic_instance
from droneplan.errors import PlannerError
from droneplan.planner import Algorithm, plan_area

BASELINE = Algorithm.DARP_STC


def run_instance(inst: Instance, algorithm: Algorithm) -> dict[str, Any]:
    row: dict[str, Any] = {
        "instance": inst.name,
        "obstacles": bool(inst.obstacles),
        "algorithm": algorithm.value,
        "drones": inst.config.drone.count,
        "stations": len(inst.stations),
        "range_km": round(inst.config.drone.range_m / 1000, 2),
    }
    try:
        plan = plan_area(inst.area, inst.stations, inst.config, algorithm, inst.obstacles)
    except PlannerError as exc:
        return {**row, "valid": False, "error": str(exc)}
    return {**row, **plan.metrics().as_dict(), "valid": not plan.violations(), "error": ""}


def _task(args: tuple[int, bool, str]) -> dict[str, Any]:
    seed, with_obstacles, algorithm = args
    return run_instance(
        synthetic_instance(seed, with_obstacles=with_obstacles), Algorithm(algorithm)
    )


def summarize(df: pd.DataFrame) -> str:
    ok = df[df["valid"]]
    base = ok[ok["algorithm"] == BASELINE.value].set_index("instance")["makespan_s"]
    ok = ok.assign(ratio=ok["makespan_s"] / ok["instance"].map(base))
    wins = ok.loc[ok.groupby("instance")["makespan_s"].idxmin(), "algorithm"].value_counts()
    table = ok.groupby("algorithm").agg(
        makespan_min=("makespan_s", lambda s: s.mean() / 60),
        vs_v1_pct=("ratio", lambda r: (r.mean() - 1) * 100),
        gap_to_lb_pct=("gap_pct", "mean"),
        turns=("n_turns", "mean"),
        solve_s=("solve_time_s", "mean"),
    )
    table["wins"] = wins.reindex(table.index).fillna(0).astype(int)
    table["valid_pct"] = df.groupby("algorithm")["valid"].mean().mul(100).round(0)
    return table.sort_values("makespan_min").round(2).to_markdown()


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="benchmarks.run")
    p.add_argument("--seeds", type=int, default=30)
    p.add_argument("--out", type=Path, default=Path("benchmarks/results"))
    p.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    args = p.parse_args(argv)
    tasks = list(product(range(args.seeds), (False, True), [a.value for a in Algorithm]))
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        df = pd.DataFrame(list(pool.map(_task, tasks)))
    args.out.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.out / "results.csv", index=False)
    sections = [
        f"## {'With' if flag else 'Without'} obstacles\n\n{summarize(df[df['obstacles'] == flag])}"
        for flag in (False, True)
    ]
    report = "\n\n".join(sections) + "\n"
    (args.out / "summary.md").write_text(report)
    print(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
