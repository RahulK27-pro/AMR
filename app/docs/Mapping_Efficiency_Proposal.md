# Mapping Efficiency — Problem Analysis & Proposed Solution

## The Root Cause of Inefficiency

Reading the `m-explore-ros2` source code directly, the cost function in
[`frontier_search.cpp`](../src/m-explore-ros2/explore/src/frontier_search.cpp) (line 192–197) is:

```
cost = (potential_scale × min_distance) − (gain_scale × frontier_size)
```

The frontier with the **lowest** cost wins. This means the algorithm:
1. **Rewards** close frontiers (`potential_scale` pulls down the cost of nearby targets).
2. **Rewards** large frontiers (more unknown cells = lower cost).
3. **Has zero spatial memory** — it never remembers which direction it just came from.

This is a pure greedy algorithm. At every replan cycle, it re-evaluates all frontiers from
scratch and picks whichever one happens to have the best cost/distance ratio at that moment.
Since SLAM updates the map continuously, frontier sizes shift constantly, and the "best"
frontier swaps back and forth. The robot ends up oscillating between two or three frontier
clusters, covering already-mapped ground over and over.

---

## Your Idea Is Correct (and is Standard Robotics Practice)

The **outer-wall-first, inward sweep** strategy you described is a well-established
technique called **Coverage Path Planning (CPP)**. It is superior to frontier exploration
for enclosed environments (warehouses, rooms) because:

- The robot discovers the full boundary first (maximum information gain per meter traveled).
- Subsequent passes are on already-constrained, predictable territory.
- It naturally avoids the oscillation problem because it follows a deterministic path.

---

## The Two Professional Solutions

### Option A — Tune `explore_lite` Cost Function (Quick, No New Code)

Modify the `frontier_search.cpp` cost function to add a **wall-proximity bonus** and a
**direction persistence bonus**. This can be approximated purely through parameter tuning:

| Parameter | Current | Proposed | Effect |
|---|---|---|---|
| `potential_scale` | 3.0 | **1.0** | Reduce distance penalty — stop being lazy about far frontiers |
| `gain_scale` | 1.0 | **5.0** | Strongly prefer *large* frontiers (wall edges are big continuous frontiers) |
| `min_frontier_size` | 0.30m | **0.50m** | Filter out tiny noise fragments, only target substantial wall frontiers |
| `planner_frequency` | 0.05 Hz | **0.033 Hz** | Longer commitment — don't let it change its mind mid-drive |

**Why this helps:** Large-size frontiers are almost always wall boundaries. By heavily
weighting `gain_scale`, the robot will automatically chase the longest unvisited wall edge
rather than the closest frontier blob. This implements the wall-preference behaviour without
writing a single line of code.

**Limitation:** It is still greedy. It will not guarantee an outward→inward spiral.

---

### Option B — Replace `explore_lite` with a Coverage Path Planner (Proper Solution)

This is the architecturally correct solution for a warehouse robot.

The approach is a **two-phase pipeline**:

**Phase 1 — Wall Contour Tracing**
A ROS 2 node subscribes to `/map` from SLAM Toolbox. As map cells are discovered, it
detects the outermost frontier (cells adjacent to unknown space AND adjacent to a wall).
It continuously feeds the robot waypoints along this frontier contour, causing the robot
to *hug the walls* and trace the full perimeter of every room.

**Phase 2 — Boustrophedon Interior Sweep**
After the perimeter is known, the enclosed free space is divided into convex cells using
**Boustrophedon decomposition** (the standard lawnmower pattern). The robot sweeps each
cell in parallel rows spaced one robot-width apart (`2 × robot_radius = 0.30m`).

```
Phase 1 (Wall Follow):       Phase 2 (Boustrophedon):
┌──────────────────┐         ┌──────────────────┐
│ →→→→→→→→→→→→→↓  │         │ →→→→→→→→→→→→→→  │
│ ↑              ↓  │         │ ←←←←←←←←←←←←←  │
│ ↑              ↓  │         │ →→→→→→→→→→→→→→  │
│ ↑              ↓  │         │ ←←←←←←←←←←←←←  │
│ ↑←←←←←←←←←←←←←  │         │ →→→→→→→→→→→→→→  │
└──────────────────┘         └──────────────────┘
  Full perimeter known          Interior swept clean
```

**Available ROS 2 Packages for Phase 2:**
- `nav2_coverage` (official Nav2 plugin, uses Fields2Cover library)
- `full_coverage_path_planner` (open source, stable on Jazzy)

---

## Recommended Implementation Plan

Given that you already have the SLAM + Nav2 stack working, the most practical path forward
is a **hybrid** approach:

### Step 1 — Immediate Improvement (Today)
Tune `explore_lite` with the wall-biased parameters from Option A. This requires
zero new code and will measurably reduce backtracking. Estimated improvement: **30–40%
reduction in total travel distance**.

### Step 2 — Custom Frontier Scorer Node (This Week)
Write a small ROS 2 node that sits between the costmap and `explore_lite`, intercepts
the frontier list, and re-scores frontiers based on:
1. **Wall proximity score:** Frontiers whose centroid is within `0.5m` of a lethal obstacle
   get their cost reduced by 40%.
2. **Directional momentum score:** The frontier in the most similar direction to the robot's
   current heading gets a 20% cost reduction (prevents U-turns).

### Step 3 — Full CPP (Next Phase)
Replace `explore_lite` entirely with a proper coverage planner. This would be Phase 2
of the project and is documented in the project roadmap.

---

## Open Questions for You

1. **Is the environment static (fixed warehouse walls)?** If yes, Option B (CPP) is
   strongly preferred. If walls change, frontier exploration is more appropriate.
2. **How large is the warehouse?** This determines whether Step 2 or Step 3 is worth the
   engineering investment.
3. **Do you want to proceed with Step 1 (parameter tuning) now**, or jump directly to
   implementing the custom frontier scorer node?
