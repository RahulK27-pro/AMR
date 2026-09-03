# Dynamic Obstacle Avoidance — Improvements & Changelog

**Date:** 2026-09-03  
**File Modified:** `src/agv_navigation/agv_navigation/route_runner.py`  
**Build Status:** ✅ Passed (`colcon build --packages-select agv_navigation`)

---

## Background & Motivation

### Observed Problem
During simulation runs logged in `route_runner_20260903_075535.log`, the robot exhibited the following pathological behaviors when navigating in the presence of dynamic obstacles:

- **Circling / spinning in place** — robot would spin for up to 12.4 seconds at a fixed location without advancing
- **Left-Right oscillation** — MPPI controller flipped angular direction every 1–2 seconds instead of committing to one side
- **False stops** — 85% of all `v=0` stops occurred when `MinObs > 0.4m` (no real obstacle nearby)
- **78 DEVIATION ALERTs** — robot drifted so far off-path that it lost its topological position
- **5–29 ghost obstacles** — LiDAR noise clusters were treated as real dynamic obstacles even when only 1 physical obstacle existed

### Root Cause Chain
```
LiDAR noise → 5–29 ghost obstacle clusters detected
    ↓
Binary evasion trigger (any obstacle anywhere) kills path-following weight
    ↓
w_cross_track drops: 6.0 → 0.5, robot loses directional anchor
    ↓
No swerve-side commitment → MPPI oscillates L/R every tick
    ↓
cos²(heading_error) law drives v_mean → 0 when heading error > 60°
    ↓
No stuck detector → robot spins indefinitely with no recovery
```

---

## Fix 1 — LiDAR Ghost Obstacle Age Filter

### Problem
Any LiDAR cluster appearing in a single scan frame was immediately counted as a dynamic obstacle. This caused 5–29 ghost readings from noise, making MPPI decisions unstable.

### Change in `scan_callback` (lines 286–324)

**Before:**
```python
new_dynamic_obs = []
if self.last_scan_time is not None and len(self.prev_obstacle_clusters) > 0:
    for curr_c in current_clusters:
        # ... match to prev ...
        if best_prev is not None:
            vel = (curr_c - best_prev) / dt
            speed = np.linalg.norm(vel)
            if 0.10 <= speed <= 2.5:     # ← single-frame is enough
                new_dynamic_obs.append({...})
self.dynamic_obstacles = new_dynamic_obs
```

**After:**
```python
# Fix 1 — LiDAR age filter: only promote clusters seen in >= 2 consecutive scans
new_dynamic_obs = []
new_age_map = {}
if self.last_scan_time is not None and len(self.prev_obstacle_clusters) > 0:
    for ci, curr_c in enumerate(current_clusters):
        # ... indexed match to prev ...
        if best_prev_idx is not None:
            vel = (curr_c - self.prev_obstacle_clusters[best_prev_idx]) / dt
            speed = np.linalg.norm(vel)
            age = self._obs_age_map.get(best_prev_idx, 0) + 1
            new_age_map[ci] = age
            if 0.10 <= speed <= 2.0 and age >= 2:   # ← requires 2+ consecutive frames
                new_dynamic_obs.append({'pos': ..., 'vel': ..., 'speed': ...,
                                        'radius': 0.35, 'age': age})
        else:
            new_age_map[ci] = 0  # new unmatched cluster, age=0
else:
    for ci in range(len(current_clusters)):
        new_age_map[ci] = 0

self._obs_age_map = new_age_map
self.dynamic_obstacles = new_dynamic_obs
```

**New `__init__` variable:**
```python
self._obs_age_map = {}   # Maps prev_cluster index -> consecutive scan count
```

**Effect:** Eliminates single-frame noise spikes. Max speed threshold also tightened from 2.5 → 2.0 m/s to reject teleporting ghost clusters.

---

## Fix 2 — Distance-Gated Evasion Trigger

### Problem
`w_cross_track` was dropped from 6.0 → 0.5 whenever ANY dynamic obstacle was detected, even those 3–4m away posing no collision risk. This caused the robot to lose its path-following anchor unnecessarily.

### Change in `control_loop` — evasion decision block

**Before:**
```python
if len(self.dynamic_obstacles) > 0:
    is_evading = True
    evasion_causes.append(f"{len(self.dynamic_obstacles)} dynamic obstacle(s)")
```

**After:**
```python
# Fix 2 — Distance-gated evasion: only relax w_cross_track for obstacles within evasion_range
close_dyn = [
    d for d in self.dynamic_obstacles
    if math.hypot(d['pos'][0] - self.current_x, d['pos'][1] - self.current_y) < self.evasion_range
]
if len(close_dyn) > 0:
    is_evading = True
    evasion_causes.append(f"{len(close_dyn)} dynamic obstacle(s) within {self.evasion_range:.1f}m")
```

**New `__init__` parameter:**
```python
self.evasion_range = 1.0   # metres — only obstacles within 1.0m collapse path-following weight
```

> **Note:** The MPPI cost rollout loop still penalises ALL dynamic obstacles regardless of distance (correct behaviour for collision avoidance). Only the `w_cross_track` relaxation is distance-gated.

---

## Fix 3 — Swerve Side Commitment

### Problem
With `w_cross_track = 0.5` in evasion mode, the MPPI cost landscape was nearly flat between left-swerge and right-swerge. Angular sampling mean of 0.0 caused the robot to flip direction every tick, making zero net progress.

### Change in `control_loop` — before `w_seq` sampling

**Before:**
```python
w_seq = np.random.normal(0.0, self.noise_w, (self.num_samples, self.horizon))
```

**After:**
```python
# Fix 3 — Swerve side commitment: bias MPPI angular sampling toward the clear side
if len(close_dyn) > 0 and is_evading:
    if self._swerve_lock_ticks <= 0:
        # Determine side using cross-product of (robot→target) × (robot→obstacle)
        nearest = min(close_dyn, key=lambda d: math.hypot(...))
        cross = dx_t * dy_o - dy_t * dx_o   # >0 = obs on left → swerve right
        self._swerve_bias = -self.swerve_bias_strength if cross > 0 else self.swerve_bias_strength
        self._swerve_lock_ticks = self.swerve_lock_duration
        # Logs: [BRAIN: SWERVE] Committed swerve RIGHT (obs on LEFT) | bias=-0.35 rad/s
    else:
        self._swerve_lock_ticks -= 1
else:
    self._swerve_bias = 0.0
    self._swerve_lock_ticks = 0

w_seq = np.random.normal(self._swerve_bias, self.noise_w, (self.num_samples, self.horizon))
```

**New `__init__` variables:**
```python
self._swerve_bias = 0.0           # Current committed angular bias (rad/s)
self._swerve_lock_ticks = 0       # Ticks remaining on current swerve direction
self.swerve_lock_duration = 20    # Hold for 20 ticks (~2s at 10 Hz)
self.swerve_bias_strength = 0.35  # Angular bias magnitude (rad/s)
```

**Log tag added:** `[BRAIN: SWERVE]`

---

## Fix 4 — Minimum Forward Velocity Floor

### Problem
The heading-velocity coupling law `v_mean = 0.35 × cos²(heading_error)` drives `v_mean → 0` when heading error exceeds 60°. Since MPPI then samples near-zero velocities, the robot stops moving forward entirely — creating a positive-feedback freeze loop where the heading error grows further.

### Change in `control_loop` — `v_mean` calculation

**Before:**
```python
v_mean = 0.35 * (heading_alignment ** 2) * approach_scale * dyn_slowdown
```

**After:**
```python
# Fix 4 — Minimum velocity floor prevents full freeze when heading error > 60°
v_mean = max(0.08, 0.35 * (heading_alignment ** 2) * approach_scale * dyn_slowdown)
```

**Effect:** Robot always samples with at least 0.08 m/s mean forward velocity. MPPI can still choose v=0 if a wall is very close (collision cost dominates), but the distribution is no longer collapsed at zero due to heading error alone.

---

## Fix 5 — Stuck Detector + Recovery Maneuver

### Problem
No mechanism existed to detect or escape a circling/spinning state. The robot could spin in place for 12+ seconds (observed: 12.4s worst case, 66° yaw swing) with zero Dijkstra reroutes triggered.

### Stuck Detector — added at end of `control_loop` (before `cmd_pub.publish`)

```python
# Fix 5 — Stuck detector: count ticks where robot spins but doesn't translate
if self._last_stuck_check_pos is not None:
    moved = math.hypot(
        self.current_x - self._last_stuck_check_pos[0],
        self.current_y - self._last_stuck_check_pos[1]
    )
    is_spinning = abs(optimal_w) > 0.15
    if moved < 0.03 and is_spinning:
        self._stuck_counter += 1
    else:
        self._stuck_counter = 0
        self._last_stuck_check_pos = (self.current_x, self.current_y)
else:
    self._last_stuck_check_pos = (self.current_x, self.current_y)

if self._stuck_counter >= self._stuck_threshold_ticks:
    # Log: [BRAIN: STUCK_DETECTOR] Stuck for N ticks at (x, y). Triggering recovery.
    self._stuck_counter = 0
    self._last_stuck_check_pos = None
    self._execute_recovery()
    return
```

### Recovery Maneuver — new `_execute_recovery()` method

```python
def _execute_recovery(self):
    """Phase 1: Back up 0.15 m/s for 1.5s → Phase 2: Dijkstra reroute"""
    now = self._now_sec()
    if self._recovery_phase is None:
        self._recovery_phase = "BACKUP"
        self._recovery_start_time = now
        # Logs: [BRAIN: RECOVERY] Phase 1/2 — BACKING UP ...

    if self._recovery_phase == "BACKUP":
        elapsed = now - self._recovery_start_time
        if elapsed < self._recovery_backup_dur:
            cmd = Twist()
            cmd.linear.x = -0.15   # reverse at 0.15 m/s
            self.cmd_pub.publish(cmd)
            return
        else:
            self._recovery_phase = "REROUTE"

    if self._recovery_phase == "REROUTE":
        # Logs: [BRAIN: RECOVERY] Phase 2/2 — REROUTING via Dijkstra ...
        self._recovery_phase = None
        self.trigger_reroute()
```

**New `__init__` variables:**
```python
self._stuck_counter = 0
self._last_stuck_check_pos = None
self._stuck_threshold_ticks = 10   # ~1s of spinning → recovery
self._recovery_phase = None        # None | 'BACKUP' | 'REROUTE'
self._recovery_start_time = None
self._recovery_backup_dur = 1.5   # seconds of reverse motion
self._last_recovery_log_time = 0.0
```

**Log tags added:** `[BRAIN: STUCK_DETECTOR]`, `[BRAIN: RECOVERY]`

---

## Summary of All New Log Tags

| Tag | When Fired |
|---|---|
| `[BRAIN: SWERVE]` | Each time a new swerve side is committed (locked for 2s) |
| `[BRAIN: STUCK_DETECTOR]` | When robot has spun ≥10 ticks without translating |
| `[BRAIN: RECOVERY] Phase 1/2` | Start of backup maneuver |
| `[BRAIN: RECOVERY] Phase 2/2` | Dijkstra reroute triggered after backup |
| `[BRAIN: MPPI_CONTROLLER]` | (Existing) Now includes distance info: "within 1.0m" |

---

## Summary of Parameter Changes

| Parameter | Old Value | New Value | Reason |
|---|---|---|---|
| Evasion trigger | `len(dynamic_obstacles) > 0` | `distance < 1.0m` | Prevent path-collapse for far obstacles |
| Max tracked speed | `2.5 m/s` | `2.0 m/s` | Reject teleporting ghost clusters |
| Min age to count | `1 frame` | `2 consecutive frames` | LiDAR ghost filter |
| `v_mean` floor | `0.0 m/s` | `0.08 m/s` | Break freeze loop at high heading error |
| `w_seq` mean | `0.0 rad/s` | `±0.35 rad/s` (biased) | Commit swerge direction |
| Stuck threshold | N/A (none) | `10 ticks (~1s)` | Trigger recovery |
| Recovery backup speed | N/A (none) | `-0.15 m/s for 1.5s` | Escape stuck position |

---

## Verification Commands

```bash
# Syntax check
python3 -m py_compile src/agv_navigation/agv_navigation/route_runner.py

# Build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agv_navigation --symlink-install

# Run with full logging
source install/setup.bash
ros2 run agv_navigation route_runner 2>&1 | tee "logs/route_runner_$(date +%Y%m%d_%H%M%S).log"

# After run — check new brain events
grep "BRAIN: SWERVE\|BRAIN: STUCK\|BRAIN: RECOVERY" logs/route_runner_*.log

# Compare obstacle count (was 5-29, should now be 0-3)
grep "LIDAR_SCAN: DYNAMIC_OBSTACLE" logs/route_runner_*.log | awk -F'Obs #' '{print $2}' | cut -d' ' -f1 | sort -n | tail -1

# Check for spin-in-place reduction (was 72, target <10)
grep -c "v=0.00m/s" logs/route_runner_*.log
```

---

## Expected Improvements After Fix

| Metric | Before | Expected After |
|---|---|---|
| Ghost obstacles per scan | 5–29 | 0–3 |
| Spin-in-place readings | 72 | < 10 |
| False halts (v=0, MinObs > 0.4m) | 61 | < 5 |
| Deviation alerts | 78 | < 20 |
| Max circling duration | 12.4s | < 1s (10 ticks) |
| MPPI evasion triggers | 503 | < 50 |
| Successful swerge-and-pass | Rare (luck) | Consistent |
