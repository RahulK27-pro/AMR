# Step 2 — Remove Camera / Vision Code

**Package:** `agv_navigation`  
**File Modified:** `src/agv_navigation/agv_navigation/route_runner.py`  
**Status:** ✅ Complete

---

## Reason

No camera hardware is present on this robot. The system uses **LiDAR-only** sensing (`/scan`).
All vision-related code was dead weight: the `/obstacle_alert` subscriber would never receive
messages, and the `vision_obstacle_detected` flag would always remain `False`.

Keeping dead code is actively harmful:
- Confuses future developers about what sensors exist
- Causes misleading log output if the topic ever receives spurious messages
- Wastes memory storing unused state variables

---

## What Was Removed

| Item | Location | Detail |
|---|---|---|
| `Point` import | `geometry_msgs.msg` | No longer needed |
| `vision_sub` | `__init__` | Subscriber to `/obstacle_alert` (camera pipeline) |
| `vision_obstacle_detected` | `__init__` | Boolean state flag — always False |
| `vision_last_time` | `__init__` | Timestamp for vision expiry — never set |
| `vision_callback()` | Method | Entire method removed |
| Vision expiry block | `control_loop` | `if vision_obstacle_detected and ...` block |
| Vision flag in cross-track | `control_loop` | Removed from `if obstacle_blocking_path or vision_obstacle_detected or ...` |

---

## What Was NOT Changed

Everything else is identical:

- Dynamic obstacle velocity estimation (LiDAR cluster tracking) — **untouched**
- Static obstacle collision/repulsion in MPPI — **untouched**
- `obstacle_blocking_path` path-proximity check — **untouched**
- Cross-track softening for LiDAR-detected obstacles — **still active** (now driven purely by LiDAR)

---

## Before vs After: Cross-Track Condition

```python
# BEFORE (with dead camera flag)
if obstacle_blocking_path or self.vision_obstacle_detected or len(self.dynamic_obstacles) > 0:
    self.w_cross_track = self.w_cross_track_evasion

# AFTER (LiDAR only — semantics identical since vision flag was always False)
if obstacle_blocking_path or len(self.dynamic_obstacles) > 0:
    self.w_cross_track = self.w_cross_track_evasion
```

---

## Build Verification

```
colcon build --packages-select agv_navigation
Summary: 1 package finished [1.73s]   ← 0 errors, 0 warnings
```

Zero remaining references to vision, camera, or obstacle_alert in route_runner.py.

---

## Note on `agv_vision` Package

The `agv_vision` package (`src/agv_vision/`) still exists in the workspace but is not
launched by any active launch file and is not used by any running node.
It can be deleted from the workspace if desired, or kept as reference code.
