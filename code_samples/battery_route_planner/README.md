# Battery-Constrained Route Planner

This is a small, self-contained extraction from my larger multi-drone coverage-planning project. It focuses on two pieces of the planning problem that compose naturally:

1. **Obstacle-aware shortest-path routing** using a visibility graph over polygon vertices.
2. **Battery-constrained route splitting** using dynamic programming.

The input is an **already ordered route**, one or more charging stations, a battery range, and optional polygonal obstacles. The output is the minimum-total-distance way to partition that fixed route into battery-feasible sorties.

## Why this exists

In the full drone planner, an earlier stage decides the order in which coverage cells should be visited. Once that route exists, it still cannot necessarily be flown on one battery charge.

A valid sortie must:

- start at a charging station,
- cover a contiguous section of the route,
- return to a charging station, and
- stay within the drone's battery range.

Straight-line distance is not sufficient when buildings or other no-fly regions block the direct path to a station. This sample therefore combines routing and route decomposition rather than treating them as independent problems.

## Example

```python
from shapely.geometry import box

from battery_route_planner import plan_sorties

result = plan_sorties(
    route=[
        [20.0, 0.0],
        [40.0, 0.0],
        [60.0, 0.0],
        [80.0, 0.0],
    ],
    stations=[
        [0.0, 30.0],
        [100.0, 30.0],
    ],
    obstacles=[box(45.0, -10.0, 55.0, 10.0)],
    battery_range_m=140.0,
)

for sortie in result.sorties:
    print(sortie)
```

The obstacle blocks the direct segment between the middle route points, so the router inserts a shortest detour around it. The dynamic program then chooses where to cut the route so every resulting flight remains within the 140 m battery range.

## How it works

### 1. Visibility-graph router

`Router` preprocesses polygonal obstacles into a visibility graph. The graph contains the convex obstacle vertices where a Euclidean shortest path can bend.

For a path query:

- if the direct segment is clear, the router returns it immediately;
- otherwise, the query endpoints are connected to visible obstacle vertices;
- shortest-path distances through the precomputed graph are used to find the best detour;
- predecessor information reconstructs the actual path.

The router also provides batched distance and nearest-station queries because the splitter needs these repeatedly.

### 2. Dynamic-programming route split

For each route point `i`, the planner precomputes:

- `prefix[i]`: cumulative obstacle-aware distance along the fixed route;
- `near[i]`: obstacle-aware distance to the nearest charging station.

The cost of covering route points `i..j` in one sortie is:

```text
station -> point i
+ route distance from i -> j
+ point j -> station
```

or equivalently:

```text
near[i] + (prefix[j] - prefix[i]) + near[j]
```

Let `best[j]` be the minimum total flight distance required to cover route points `0..j-1`. For every possible final sortie `i..j`, the algorithm relaxes `best[j+1]` when that sortie fits within the battery range.

This takes **O(n²)** time after routing distances are available.

## Correctness

The tests include a brute-force oracle for small route instances. It enumerates every possible placement of route cuts and compares the true optimum with the dynamic-programming result.

This is useful because it checks the optimization algorithm itself rather than only checking a handful of hand-written examples.

The test suite also covers:

- direct routing without obstacles,
- shortest detours around obstacles,
- obstacle-aware nearest-station selection,
- multi-station route splitting,
- an obstacle changing a previously feasible battery decision, and
- DP optimality against brute force.

## Setup

Requires Python 3.11+.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

Run the example:

```bash
python examples/demo.py
```

Run the tests:

```bash
pytest
```

## Project structure

```text
src/battery_route_planner/
├── __init__.py      # small public API
├── errors.py        # domain-specific exceptions
├── planner.py       # DP splitter and high-level plan_sorties API
└── router.py        # visibility graph and shortest-path queries

examples/
└── demo.py

tests/
├── test_planner.py
└── test_router.py
```

## Scope and tradeoffs

This sample intentionally assumes the route order is already known. It does **not** solve the full coverage-routing or multi-drone scheduling problem.

The dynamic program is optimal only for the following problem:

> Given a fixed route order, charging-station locations, obstacle-aware distances, and a maximum flight distance, find the feasible partition with minimum total flight distance.

The visibility graph also trades preprocessing cost for fast repeated routing queries. Its all-pairs obstacle-vertex preprocessing grows roughly quadratically in the number of relevant vertices, which is appropriate for the original application where the same obstacle map is queried many times, but it would need a different strategy for very large obstacle sets.

## Relationship to the full project

This code was extracted from `drone-coverage-planner`, a larger system that:

1. rasterizes a geographic area at the camera footprint,
2. constructs an ordered coverage tour,
3. splits that tour into battery-feasible sorties,
4. schedules sorties across multiple drones and charging stations, and
5. validates and visualizes the final plan.

This sample stops deliberately after step 3 so the routing and optimization logic can be reviewed without requiring the rest of the application context.
