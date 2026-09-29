# droneplan

Plans the fastest way for a fleet of battery-limited drones to map an area, recharging at automatic stations and flying around obstacles. Built from the ISRO problem statement (Smart India Hackathon 2020).

## Quick start
```bash
python3.12 -m venv .venv && source .venv/bin/activate && pip install -e ".[dev]"
droneplan area.shp --stations stations.geojson --obstacles buildings.shp \
  --drones 4 --range-km 10 --speed 15 --out plan.geojson
```

## Run the app
```bash
docker compose up --build      # then open http://localhost:8000
```
Or for development: `pip install -e ".[dev,api]" && uvicorn droneplan.api.app:app --reload`, and in another terminal `cd web && pnpm install && pnpm dev` (http://localhost:5173).

Upload an area (`examples/area.geojson`) and obstacles (`examples/obstacles.geojson`), add charging stations on the map, and click **Plan**. The replay shows each drone's position, state (flying, queued, charging) and battery, a schedule Gantt chart, and the metrics next to the v1 baseline.

## How it works
1. **Route:** lanes along the minimum-width sweep angle, ordered by turn-aware nearest-neighbor + 2-opt. A visibility graph routes around obstacles.
2. **Split:** dynamic programming chooses optimal battery-feasible cut points.
3. **Schedule:** longest-first list scheduling with charging queues, plus a search over the sortie budget.
4. **Validate:** one independent checker scores every algorithm, including the baselines.

## Results (30 seeded synthetic areas, each with and without obstacles)
### Without obstacles

| algorithm          |   makespan_min |   vs_v1_pct |   gap_to_lb_pct |   turns |   solve_s |   wins |   valid_pct |
|:-------------------|---------------:|------------:|----------------:|--------:|----------:|-------:|------------:|
| rss                |         256.3  |      -41.93 |          101.68 |  134.13 |      0.64 |     30 |         100 |
| rss-full-budget    |         262.2  |      -39.85 |          110.44 |  138.67 |      0.06 |      0 |         100 |
| rss-fixed-angle    |         271.44 |      -37.34 |          125.05 |  167.33 |      0.33 |      0 |         100 |
| rss-greedy-split   |         279.01 |      -37.04 |          117.17 |  133.27 |      0.38 |      0 |         100 |
| darp-stc           |         416.43 |        0    |          315.46 |  598.63 |      0.15 |      0 |         100 |
| darp-boustrophedon |         421.94 |       -1.09 |          296.52 |  301.43 |      0.13 |      0 |         100 |

### With obstacles

| algorithm          |   makespan_min |   vs_v1_pct |   gap_to_lb_pct |   turns |   solve_s |   wins |   valid_pct |
|:-------------------|---------------:|------------:|----------------:|--------:|----------:|-------:|------------:|
| rss                |         255.55 |      -40.79 |          110.2  |  144.43 |      1.25 |     30 |         100 |
| rss-full-budget    |         260.38 |      -39.33 |          116.75 |  148.17 |      0.29 |      0 |         100 |
| rss-fixed-angle    |         271.02 |      -36.85 |          127.38 |  183.4  |      0.67 |      0 |         100 |
| rss-greedy-split   |         278.57 |      -35.73 |          128.06 |  149.9  |      0.8  |      0 |         100 |
| darp-stc           |         412.97 |        0    |          304.45 |  615.53 |      0.22 |      0 |         100 |
| darp-boustrophedon |         418.34 |        1.51 |          310.25 |  316.5  |      0.18 |      0 |         100 |

`vs_v1_pct`: mean change in makespan vs the v1 pipeline (DARP + STC + greedy refuel). `gap_to_lb_pct`: how far above a provable lower bound. `solve_s`: the whole `plan_area` call, including projection and router build. `rss` plans at both the min-width sweep angle and 0°, with both the DP and greedy split, and keeps the lowest makespan; each `rss-*` ablation removes one of those choices, so none can beat `rss`.

The baselines (`darp-stc`, `darp-boustrophedon`) are re-implementations of v1 with its bugs fixed, and they share the same router and validator, so the comparison is fair. The comparison is on makespan, not optimality. The lower bound charges each mappable cell min(s, 2d) of flying, where s is the cell size and d the routed distance from the cell to its nearest station, plus pro-rata recharging of the energy beyond the fleet's full batteries; it ignores turns and most of the flying to and from stations, so it stays loose: `rss` averages 101.68% above it without obstacles and 110.2% with them, `darp-stc` 315.46% and 304.45%. A large `gap_to_lb_pct` therefore does not mean a plan is far from optimal. The bound is computed on each algorithm's own grid, so `gap_to_lb_pct` for the fixed-angle algorithms uses a different bound.

**Limits:** the visibility graph is built over all pairs of obstacle vertices, so its cost grows with their square (about 4 s at ~800 nodes, ~64 s at ~1800); very large building layers need simplifying first.

## Development
`pytest` · `ruff check .` · `mypy src tests benchmarks` · `python -m benchmarks.run --seeds 30`
