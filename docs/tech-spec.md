# Tech Spec — Multi-Drone Area Mapping Planner

**Status:** v3 · core planner implemented, API and web replay implemented · **Author:** Yash Bitla · **Origin:** ISRO problem statement, Smart India Hackathon 2020

---

## 1. Problem

> Develop an application for automatically planning a route (for shortest time to cover area) and schedule of drones for mapping a given area. Inputs: (1) map of area (shapefile), (2) number of drones, (3) range in km based on battery life, (4) top speed, (5) locations of automatic charging stations. The software must visualize a simulated animation of the plan.

**Objective:** minimize **makespan**, the time until the last drone lands with every mappable cell covered.

**Constraints**
- Every sortie starts and ends at a charging station and stays within the usable range.
- No flight path passes through an obstacle that is taller than the flight altitude.
- A station charges one drone at a time. Other drones wait on the ground.
- Drones start at the stations, assigned round-robin.

**Inputs**
- The five from the problem statement.
- An **optional obstacle layer**: polygons, each with an optional `height_m`.

**Outputs**
- A path per flight
- A schedule: launch, land, charge, and wait times
- Metrics
- An animated replay

### Assumptions (all configurable)

| Parameter | Default | Why |
|---|---|---|
| Flight altitude and swath `w` | 100 m, FOV 60°, 20% overlap → **92.4 m** | Not in the problem. Mapping needs overlap. |
| Charging time | 30 min from empty to full, linear in energy used | Not in the problem. |
| Station capacity | 1 pad | Realistic for automatic pads. |
| Turn penalty | `(v/a)·(1−cos Δθ)/2`, with a = 2 m/s² | A full reversal loses `v/a` s. Flying straight loses 0. |
| Energy | Proportional to distance, 10% reserve | Range is given in km. |
| Obstacle clearance | 20 m horizontal buffer | Safety margin around buildings and terrain. |
| Out of scope | Wind, drone-to-drone collision (we assume altitude separation), climbing over obstacles | Keeps the problem 2-D. |

### Two kinds of "not the area"

| Kind | Example | Must map it? | Can fly over it? |
|---|---|---|---|
| **Hole / outside the area** | A lake excluded from the survey | No | **Yes** |
| **Obstacle** (height ≥ altitude − clearance, or no height given) | Tower, ridge, mountain | No: it's reported as *unmappable* | **No**. Paths route around it. |

Obstacles that are shorter than the flight altitude are ignored, because the drone flies over them and maps them. Mountains are given as the contour polygon at flight altitude. Reading a DEM directly is a stretch goal.

---

## 2. Where the v1 approach falls short

v1 pipeline: **DARP** (equal-area split) → **STC** (spanning-tree path) → **greedy refuel detours**.

| Issue | Effect |
|---|---|
| DARP balances **area**, not **time** | A drone whose region is far from a station spends more time flying back to recharge, so it sets the makespan. |
| Refueling added after the path is built | Greedy, sub-optimal, and can get stuck. |
| One notion of blocked: every hole is a wall | Routes around lakes a drone could fly over. And real obstacles can't be expressed separately. |
| Obstacle avoidance by grid BFS | Staircase paths. Every step of the staircase is a turn, and turns cost time. |
| STC turns at almost every cell | Each turn costs `v/a` s at top speed. |
| No schedule | Charging time and station queues are ignored. |
| Bugs | Unreachable cells treated as distance 0. DARP crashes when a drone gets 0 cells. The continuity check allows diagonal-only regions. N² memory. Projection done in degrees. |

---

## 3. Approach: Route → Split → Schedule (RSS)

**Idea:** build one fast coverage tour, cut it optimally into battery-feasible sorties, and schedule the sorties across drones to minimize makespan.

This is route-first, cluster-second (Beasley 1983; Prins 2004). The min-max form, tour splitting, has a proven approximation bound (Frederickson, Hecht & Kim 1978).

```
Shapefile + obstacles → [0] Frame, grid, router → [1] Tour → [2] Split → [3] Schedule → [4] Validate
```

### Stage 0: Frame, grid, and router
- **Frame:** project to local **UTM** (meters), then rotate by the sweep angle. Degrees would distort east–west distances by `cos(lat)`, which is 5–15% in India.
- **Grid:** rasterize into `w × w` cells. A cell is *required* if any part of it overlaps the area. It's *blocked* if any part of it overlaps an inflated obstacle.
  - Mappable = required and not blocked.
  - Blocked area is reported as unmappable.
  - The segment between two neighboring mappable cells in a lane is **guaranteed obstacle-free**, so paths inside lanes need no checks.
- **Router:** this is the one place distances are computed. All later stages call it.
  - **No obstacles:** straight-line (Euclidean) distance. This is the fast path.
  - **With obstacles:** a **visibility graph**.
    1. The nodes are the convex vertices of the inflated obstacles.
    2. Two nodes are connected if the straight segment between them is clear.
    3. Precompute shortest paths between all pairs of nodes (Dijkstra, via SciPy).
    4. For a query point, connect it to the nodes it can see. Then `d(p,q) = |pq|` if the direct line is clear, otherwise `min |pa| + D[a,b] + |bq|`.
    5. Clear/blocked tests are vectorized with a Shapely STRtree.

**Why a visibility graph:**

| Alternative | Why not |
|---|---|
| Grid A\* / BFS (v1) | Staircase paths with many turns (slow under our turn model). Cost grows with the number of grid cells, not obstacle complexity. |
| RRT\* / PRM | Random, gives different results on each run, and only asymptotically optimal. They're built for high-dimensional spaces, not 2-D polygons. |
| **Visibility graph** | **Exact** shortest paths among polygons. Paths bend only at obstacle corners, so there are few turns. Size depends on obstacle vertices, not map size. |

### Stage 1: Coverage tour
1. **Sweep angle:** lanes run parallel to the convex-hull edge of **minimum width**. That gives the fewest lanes and so the fewest turns (rotating calipers; Huang 2001). Fewest turns is not always the lowest makespan, so `rss` also plans at angle 0 and keeps the better plan.
2. **Lanes:** each maximal run of mappable cells in a row. Obstacles split lanes automatically.
3. **Order and direction:** nearest-neighbor + **2-opt** over a cost matrix of lane ends. Each entry is `router distance + turn time × v`, with turns taken from the actual detour's first and last legs.
   - The matrix is symmetric, so each 2-opt move costs O(1) to evaluate.
   - The same kernel works with or without obstacles.
4. The tour polyline includes detour waypoints between lanes.

| Alternative | Why not |
|---|---|
| STC | 2×2 blocks, so partial blocks need hacks (400 lines in v1), and it turns at nearly every cell. |
| Wavefront | Many turns, and backtracking. |
| BCD | Decomposes around obstacles in the *grid*. The router already handles obstacles, so lanes plus ordering is simpler and just as good. |
| Exact coverage TSP (MILP) | Only feasible for about 100 lanes. We report a lower-bound gap instead. |

### Stage 2: Optimal sortie split (dynamic programming)
- A sortie covers tour points `pᵢ…pⱼ`, and its length is `near(pᵢ) + L(i..j) + near(pⱼ)`, which must be at most the budget `B`.
  - `near` = router distance to the nearest station.
  - `L` = length along the tour, including detours.
- The DP finds the cut points with minimum total distance (energy). That also minimizes total charging time, since charging is linear in energy.
- Complexity `O(m·k)`, compiled with Numba.
- **Why:** it's **provably optimal** for a given tour and budget. Greedy "refuel when low" isn't.
- Optimal energy is not always the lowest makespan, so `rss` also runs the greedy split, with its own budget search, and keeps the better schedule.

### Stage 3: Schedule, plus a search over the budget
- **List scheduling:**
  - A free drone takes the longest remaining sortie it can reach, preferring sorties whose home station is where it already is.
  - It lands at the station where it will finish charging soonest, counting the queue.
  - If it can't reach any sortie, it repositions to another station.
  - Launch and landing legs use router paths.
- **Budget search:** 12 log-spaced values of `B`, then golden-section refinement. Keep the lowest makespan.
  - A small `B` gives more parallelism but more flying back and forth to stations.
  - A large `B` gives fewer sorties but leaves drones idle.
- **Why:** longest-first has a `4/3` bound for identical drones. The budget is the main lever. General local search was dropped because it adds complexity for little expected gain (YAGNI).

### Stage 4: Validator
One checker for every algorithm, including the baselines:
- coverage (checked against cell centers actually on a path)
- range
- station endpoints
- **no path segment inside an obstacle**
- timing matches the kinematics model
- no drone double-booked
- pad capacity respected

The same timeline drives the animation.

---

## 4. How we prove it's better

| ID | Pipeline |
|---|---|
| `darp-stc` | v1 reproduction: DARP + STC + greedy refuel, with bugs fixed and using the same router |
| `darp-boustrophedon` | DARP + our lane tour per region + greedy refuel |
| `rss` | **Ours** |
| `rss-fixed-angle` / `rss-greedy-split` / `rss-full-budget` | Ablations. Each removes one choice from `rss`'s search, so none can beat it. |
| *Stretch* | OR-Tools VRP over lanes, as a general-purpose solver for comparison |

- **Metrics:**
  - Makespan (primary)
  - Gap to the lower bound
  - Total distance
  - Turns
  - Flights
  - Queue wait
  - Utilization
  - Coverage %
  - Redundancy %
  - Unmappable area
  - Solve time
- **Lower bound:** `max(workload, pad capacity, reach)`. Workload charges each mappable cell `min(s, 2d)` of flying, where `d` is its routed distance to the nearest station, plus recharging of the energy beyond the fleet's full batteries. All terms use router distances, so the bound stays valid with obstacles.
- **Test data:** 30 or more seeded synthetic areas, run **with and without** random obstacles (1–4 stations, 2–8 drones, 6–20 km range, raised where needed so the farthest point is reachable; the largest is 29.6 km). Each area's obstacle variant shares its stations and fleet, so the two runs differ only in the obstacles. Real shapefiles are used in the demo.

---

## 5. System design

| Layer | Choice | Why |
|---|---|---|
| Core | Python 3.12, NumPy, **Numba** (2-opt, split DP), SciPy (csgraph Dijkstra), Shapely 2 (STRtree), pyproj, GeoPandas/pyogrio | Known stack. Numba where loops dominate. |
| API | FastAPI + Pydantic, with in-memory jobs in a process pool | Typed, with automatic docs. No Redis needed. |
| Web | React + TypeScript + Vite, react-leaflet | Map with uploaded area and obstacles and click-to-add stations, timeline replay with per-drone state and battery, SVG schedule Gantt, metrics table against the v1 baseline. |
| Quality | pytest + Hypothesis, Vitest, ruff, mypy (strict), GitHub Actions | Invariants checked on random maps with random obstacles. |
| Run | Docker Compose | One command. |

```
src/droneplan/
  config.py  errors.py  kinematics.py  io.py  planner.py  bounds.py  metrics.py  validation.py  export.py  cli.py
  geometry/    frame.py · sweep.py · grid.py · obstacles.py · routing.py
  coverage/    lanes.py · tour.py · _kernels.py        # lanes + nearest-neighbor/2-opt ordering
  sortie/      split.py · _kernels.py                  # split DP
  scheduling/  model.py · charging.py · flying.py · list_scheduler.py · fixed_scheduler.py
  solvers/     rss.py · area_first.py                  # full pipelines: ours and the baselines
  baselines/   darp.py · stc.py · greedy_refuel.py     # v1 building blocks, re-implemented
  api/         app.py · jobs.py · schemas.py           # FastAPI service
benchmarks/    datasets.py · run.py · results/
web/           React + Vite app
```

The package is **`droneplan`**, named after what it does. v1's `cpp_algorithms` ("coverage path planning") described only one stage of the pipeline. The layout is organized by pipeline stage, so the directory tree reads in the same order as §3.

---

## 6. Code conventions

The code should read like carefully written, human-owned code:

- **Comments only for the non-obvious:** a reference (for example "Prins 2004 split"), an invariant, or why a less-obvious choice was made. Never comments that restate what a line does.
- **Docstrings only where they add something:** public entry points, and functions with a contract that isn't clear from the signature (units, ranges, what a return value means). Each one is specific; no boilerplate "Args/Returns" blocks. Small helpers get none.
- **Clear names over explanations:** `usable_range_m`, `charge_end`, `near_station_dist`.
- **Small modules with one job each.** Typed signatures. No dead code or speculative flags.
- **Commits** are small and scoped, with plain messages. Each one works on its own.

---

## 7. Repository

- Feature branches merge into `main` through pull requests. CI runs ruff, mypy (strict) and pytest.

---

## 8. Risks

| Risk | Mitigation |
|---|---|
| Tour ordering is a heuristic | Report the lower-bound gap. The ablations isolate each stage. |
| Makespan isn't unimodal in `B` | Grid scan first, then local refinement. |
| Complex obstacles blow up the visibility graph | Simplify obstacles (tolerance about `w/4`), then grow them by the tolerance too so the clearance never shrinks. Keep only convex vertices. Graph size is logged. |
| Obstacles near stations make sorties infeasible | Stations inside obstacles are rejected with a clear error. Unreachable cells are reported, not silently dropped. |
| The advantage shrinks at large range | Show results across ranges. |
