# Step 1 — Multi-Goal Waypoint Following

**Package:** `agv_navigation`  
**File Modified:** `src/agv_navigation/agv_navigation/route_runner.py`  
**Status:** ✅ Implemented & Built

---

## What Was Added

Previously the robot could only navigate to **one goal at a time** — set from RViz's `2D Goal Pose` tool. Every new RViz click would abort the current path and replan.

This step adds a **mission queue layer** on top of the existing Dijkstra + MPPI navigation stack. You can now send an ordered list of graph node IDs, and the robot will navigate to each in sequence without manual intervention.

---

## Changes Made

### 1. New subscriber: `/goal_sequence`

```
Topic   : /goal_sequence
Type    : std_msgs/String
Payload : JSON array of graph node IDs, e.g. '["N5", "N12", "N40"]'
```

The callback (`sequence_callback`) validates all node IDs against the loaded graph, then builds an ordered `(x, y)` queue.

### 2. New publisher: `/mission_progress`

```
Topic   : /mission_progress
Type    : std_msgs/String
Payload : JSON object updated at every goal transition
```

Example payload:
```json
{"current": 2, "total": 3, "goal_node": "N12", "state": "NAVIGATING"}
```

States:
| Value | Meaning |
|---|---|
| `NAVIGATING` | En-route to a goal |
| `YIELDING` | Stopped for a crossing obstacle |
| `IDLE` | Between goals |
| `MISSION_COMPLETE` | All goals visited |

### 3. Internal queue state

Three new instance variables:
- `self.goal_queue` — ordered list of `(x, y)` tuples remaining to visit
- `self.mission_total` — total number of goals sent in this mission
- `self.mission_current` — 1-based index of the goal currently executing

### 4. Arrival logic extended

When the robot reaches a destination (within 0.30m):
- If `goal_queue` is **not empty** → dequeue next, replan, resume
- If `goal_queue` is **empty** → stop, publish `MISSION_COMPLETE`, reset counters

---

## Backward Compatibility

| Feature | Status |
|---|---|
| RViz `2D Goal Pose` click | ✅ Unchanged |
| `/goal_pose` topic | ✅ Unchanged |
| `/move_base_simple/goal` topic | ✅ Unchanged |
| Dynamic obstacle avoidance | ✅ Unchanged — MPPI runs on every segment |
| Yield & re-routing | ✅ Unchanged — works on every segment of a mission |

---

## Flow Diagram

```
ros2 topic pub /goal_sequence  '["N5","N12","N40"]'
         |
         v
sequence_callback()
 |-- validate all node IDs
 |-- build goal_queue = [(x5,y5), (x12,y12), (x40,y40)]
 |-- mission_total = 3, mission_current = 0
 +-- _dispatch_next_goal()
         |
         v
Goal 1 -> goal_callback((x5,y5)) -> Dijkstra -> MPPI -> /cmd_vel
         | (arrival within 0.30m)
         v
Goal 2 -> goal_callback((x12,y12)) -> Dijkstra -> MPPI -> /cmd_vel
         | (arrival within 0.30m)
         v
Goal 3 -> goal_callback((x40,y40)) -> Dijkstra -> MPPI -> /cmd_vel
         | (arrival within 0.30m)
         v
MISSION_COMPLETE published to /mission_progress. Robot stops.
```

---

## How to Test

```bash
# Terminal 1
ros2 launch agv_description navigation_launch.py

# Terminal 2
ros2 run agv_navigation route_runner

# Terminal 3 - watch progress
ros2 topic echo /mission_progress

# Terminal 4 - send a 3-goal mission
ros2 topic pub --once /goal_sequence std_msgs/String \
  'data: "[\"N5\", \"N12\", \"N40\"]"'
```

### How to find valid node IDs

```bash
python3 -c "
import json
d = json.load(open('src/agv_description/maps/warehouse_graph.json'))
print([n['id'] for n in d['nodes'][:10]])
"
```

---

## Known Limitation

If a goal node is completely unreachable (blocked edges), `calculate_dijkstra` returns an empty path and the mission halts. Skipping unreachable goals and advancing the queue will be addressed in the Behavior Tree phase.
