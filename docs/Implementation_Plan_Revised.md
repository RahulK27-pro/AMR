# AMR Project — Revised Implementation Plan

**Last updated:** 2026-08-30  
**Status key:** ✅ Done | 🔄 In Progress | ⏳ Pending

---

## Scope Clarification

- **No camera hardware** — `agv_vision` package and all camera/vision topics are removed from active use.
- **LiDAR-only sensing** — obstacle detection is handled entirely via `/scan` (LiDAR).
- **Full graph density preserved** — 300+ node topological graph remains untouched.

---

## Completed Items

| Step | Feature | Files | Status |
|---|---|---|---|
| Fixes | sim-time, hardcoded paths, costmap static_layer, pkg deps, docs | multiple | ✅ Done |
| Step 1 | Multi-Goal Waypoint Following (`/goal_sequence` + `/mission_progress`) | `route_runner.py` | ✅ Done |
| Step 2 | Remove Camera / Vision Code (LiDAR-only cleanup) | `route_runner.py` | ✅ Done |
| Step 3 | Phase 5: KD-Tree + MPPI Spatial Pre-Filter | `route_runner.py` | ✅ Done |
| Step 4 | Behavior Tree Orchestration (`bt_manager.py`) | `bt_manager.py` | ✅ Done |

---

## Step 2 — Remove Camera / Vision Code ✅ Done

**Goal:** Strip all camera and vision references from `route_runner.py` since no camera is present.

### Files to Modify
- `src/agv_navigation/agv_navigation/route_runner.py`

### Changes
- Remove `vision_sub` subscriber (`/obstacle_alert`)
- Remove `vision_obstacle_detected` and `vision_last_time` state variables
- Remove `vision_callback()` method
- Remove vision early-warning expiry block in `control_loop`
- Remove `vision_obstacle_detected` from cross-track softening condition
- Remove `Point` import (from `geometry_msgs.msg`) if no longer needed

### Result
Route runner becomes LiDAR-only, with no dead code, no unused subscribers,
and no references to a sensor that doesn't exist.

---

## Step 3 — Phase 5: KD-Tree + MPPI Spatial Pre-Filter ✅ Done

**Goal:** Optimize the two hotspots in `route_runner.py` without changing graph density.

### 3.1 KD-Tree Fast Node Snapping — `find_closest_node()`

Currently O(N) linear scan over 300+ nodes on every goal reception and re-routing event.

```python
from scipy.spatial import KDTree

# At graph load time:
self.node_ids = list(self.map_data.keys())
self.node_coords = np.array([[self.map_data[n]["x"], self.map_data[n]["y"]] for n in self.node_ids])
self.node_kdtree = KDTree(self.node_coords) if len(self.node_coords) > 0 else None

# In find_closest_node(x, y):
if self.node_kdtree is None or len(self.node_ids) == 0:
    return None
_, idx = self.node_kdtree.query([x, y])
return self.node_ids[idx]
```

**Benefit:** O(log N) lookup — sub-millisecond regardless of graph size.

### 3.2 MPPI Spatial Pre-Filter — static obstacle loop

Currently evaluates ALL LiDAR obstacle points at every horizon step.
At 10 Hz with 120 obstacle points × 15 horizon steps × 80 samples = ~144,000 distance ops/tick.

```python
# Pre-filter: only obstacles within 3.5m of the robot
obs_dists_sq = (self.obstacles[:, 0] - self.current_x)**2 + \
               (self.obstacles[:, 1] - self.current_y)**2
near_mask = obs_dists_sq < 12.25  # 3.5^2 — avoids sqrt
near_obs = self.obstacles[near_mask]

if len(near_obs) > 0:
    obs = near_obs[np.newaxis, :, :]
    for t in range(self.horizon):
        pts = np.stack([x_rollout[:, t], y_rollout[:, t]], axis=-1)[:, np.newaxis, :]
        min_dists = np.min(np.linalg.norm(pts - obs, axis=-1), axis=1)
        ...
```

**Benefit:** In open corridors (most common case), only ~10-30% of points are within 3.5m,
reducing matrix ops by 70-90% per tick.

### Files to Modify
- `src/agv_navigation/agv_navigation/route_runner.py`

---

## Step 4 — Behavior Tree Orchestration ✅ Done

**Goal:** Replace the `IDLE → PLANNING → NAVIGATING → YIELDING` state machine with
a formal `py_trees` Behavior Tree for maintainability and extensible recovery.

### Install
```bash
pip3 install py-trees
sudo apt install ros-jazzy-py-trees-ros ros-jazzy-py-trees-ros-interfaces
```

### New file: `src/agv_navigation/agv_navigation/bt_manager.py`

Tree structure:
```
Root (Sequence)
├── Condition: Is AMCL localized? (map→base_link TF exists)
├── Condition: Has goal been set?
├── Action: Plan path (Dijkstra)
├── Selector
│   ├── Sequence (normal)
│   │   ├── Action: MPPI FollowPath
│   │   └── Condition: Arrival check (dist < 0.3m)
│   └── Sequence (recovery)
│       ├── Action: BackUp (0.3m reverse)
│       ├── Action: Spin (60° rotate)
│       └── Action: Re-plan
└── Action: Mission Complete
```

Also adds: skip unreachable goals in mission queue (plugs the known limitation from Step 1).

---

## Step 5 — FastAPI Mobile Bridge (NEXT)

**Goal:** Create a new `agv_api` ROS 2 package with FastAPI + WebSocket server for
remote goal dispatch and real-time telemetry streaming.

### New package: `src/agv_api/`

| Endpoint | Method | Description |
|---|---|---|
| `/health` | GET | Service status |
| `/goal` | POST | Send single goal node ID |
| `/sequence` | POST | Send ordered list of node IDs (mission) |
| `/stop` | POST | Emergency stop |
| `/status` | GET | Current pose, state, mission progress |
| `/graph` | GET | Full topological graph JSON |
| `/ws` | WebSocket | Real-time telemetry (10 Hz stream) |

```bash
pip3 install fastapi uvicorn websockets
```

---

## Execution Order

| Step | Description | Effort | Dependency |
|---|---|---|---|
| **Step 2** | Remove vision/camera code | 30 min | None — do first |
| **Step 3** | KD-Tree + MPPI pre-filter | 1 hr | Step 2 done |
| **Step 4** | Behavior Tree | 3-4 days | Step 3 done |
| **Step 5** | FastAPI bridge | 2-3 days | Step 4 done |
