# Step 4 — Behavior Tree Orchestration

**Package:** `agv_navigation`  
**File Added:** `src/agv_navigation/agv_navigation/bt_manager.py`  
**Status:** ✅ Complete & Built

---

## Overview

Step 4 introduces formal **Behavior Tree Orchestration** using `py_trees` via a dedicated node `bt_manager`. 

Instead of relying on monolithic state loops, `bt_manager` organizes mission lifecycle, localization verification, goal dispatch, and recovery behaviors into a modular, hierarchical tree structure.

---

## Architecture & Tree Structure

```
AGV_BT_Root (Sequence)
├── Check_Localization (Condition: verifies map -> base_link TF)
├── Check_Goal_Queue (Condition: checks active goal or mission queue)
├── Plan_Topological_Path (Action: dispatches goal to planner)
└── Nav_Or_Recovery (Selector)
    ├── Execute_MPPI_Nav (Sequence: monitors MPPI navigation status)
    └── Recovery_Sequence (Sequence)
        ├── Backup_Action (Action: 0.3m reverse velocity)
        └── Spin_Action (Action: 60° rotation to clear sensor view)
```

---

## Key Implementations

1. **`CheckLocalization`**: Verifies map-to-base_link TF availability before initiating navigation.
2. **`CheckGoalQueue`**: Monitors goal availability from `/goal_pose` or `/goal_sequence`.
3. **`PlanTopologicalPath`**: Dispatches goals to the topological planner.
4. **`ExecuteMPPINavigation`**: Tracks active MPPI execution status from `/mission_progress`.
5. **Recovery Behaviors**:
   - `BackUpRecovery`: Reverses AGV briefly when trapped in tight dynamic blockages.
   - `SpinRecovery`: Rotates AGV to refresh local LiDAR view.

---

## Execution & Verification

### Build Status
- Compiled clean with `colcon build --packages-select agv_navigation`.

### How to Run

```bash
# Terminal 1: Launch Navigation Simulation
ros2 launch agv_description navigation_launch.py

# Terminal 2: Run Route Runner
ros2 run agv_navigation route_runner

# Terminal 3: Run Behavior Tree Manager
ros2 run agv_navigation bt_manager
```
