# Dynamic Obstacle Guide: Adding & Avoiding Moving Obstacles

This guide explains how to add, configure, and test custom **dynamic obstacles** (human workers, motorized carts, moving pallets) in Gazebo Harmonic, and how the Autonomous Mobile Robot (AMR) predicts, avoids, yields to, and dynamically re-routes around them.

---

## 1. Quick Start: Testing Dynamic Obstacles

### Step 1: Launch the Navigation Simulation
In **Terminal 1**, start Gazebo Harmonic and localization:
```bash
source ~/AMR/AMR-main/install/setup.bash
ros2 launch agv_description navigation_launch.py
```

### Step 2: Start the Upgraded Route Runner
In **Terminal 2**, start the navigation node:
```bash
source ~/AMR/AMR-main/install/setup.bash
ros2 run agv_navigation route_runner
```

### Step 3: (Optional) Visualize Graph in RViz
In **Terminal 3**, render the topological graph:
```bash
source ~/AMR/AMR-main/install/setup.bash
ros2 run agv_navigation graph_visualizer
```

### Step 4: Spawn a Dynamic Obstacle
In **Terminal 4**, spawn a dynamic obstacle into the warehouse world:

```bash
# Pattern 1: Aisle Crossing (Walks back and forth across an aisle)
ros2 launch agv_description dynamic_obstacle.launch.py pattern:=aisle_crossing speed:=0.35 x:=1.8 y:=0.0

# Pattern 2: Doorway Blocker (Blocks central doorway to test automatic re-routing)
ros2 launch agv_description dynamic_obstacle.launch.py pattern:=doorway_blocker x:=3.0 y:=2.8

# Pattern 3: Interactive Manual Teleoperation (Drive the obstacle with keyboard)
ros2 launch agv_description dynamic_obstacle.launch.py run_controller:=false x:=1.5 y:=0.0
```

When running **Pattern 3** (interactive mode), open a new terminal and drive the obstacle with **WASD**:
```bash
source ~/AMR/AMR-main/install/setup.bash
ros2 run agv_navigation obstacle_teleop
```

**Controls:**
- `[W]`: Move Forward
- `[S]`: Move Backward / Reverse
- `[A]`: Turn Left
- `[D]`: Turn Right
- `[Q] / [E]`: Forward-Left / Forward-Right
- `[Z] / [C]`: Reverse-Left / Reverse-Right
- `[Space] / [X]`: Full Stop
- `[+] / [-]`: Increase / Decrease Speed

---

## 2. How the AMR Avoids Dynamic Obstacles

The AMR navigation stack employs a **3-tier hierarchical response** to moving obstacles:

### Tier 1: Predictive MPPI Rollout (Moving Obstacle Evasion)
- The AMR's LiDAR (`/scan`) clusters moving obstacle points and estimates their 2D velocity vector $(v_x, v_y)$.
- In MPPI, the obstacle's future position is rolled out over the 1.5-second horizon:
  $$\mathbf{p}_{\text{obs}}(t) = \mathbf{p}_{\text{obs}}(0) + t \cdot \Delta t \cdot \mathbf{v}_{\text{obs}}$$
- **Graduated Repulsive Potential Field**: A smooth repulsive cost activates within 0.85m of the obstacle, allowing the robot to proactively curve around moving obstacles rather than waiting until it reaches the collision radius.
- **Adaptive Centerline Slack**: When an obstacle is detected in front, the cross-track penalty softens from $6.0 \to 2.5$, allowing the robot to veer off the path centerline to pass.

### Tier 2: Yield & Wait (Narrow Aisle Crossing)
- In narrow warehouse aisles (e.g. 1.2m–1.5m wide), steering around a dynamic obstacle without scraping warehouse shelves is physically impossible.
- If an obstacle blocks the path directly ahead and all MPPI rollouts detect a collision:
  1. The AMR decelerates smoothly and enters the `YIELDING` state (`v = 0.0 m/s`).
  2. It waits for the crossing obstacle (human or cart) to pass.
  3. Once the aisle clears, the robot automatically resumes navigation to the goal.

### Tier 3: Dynamic Dijkstra Re-Routing (Persistent Blockages)
- If an obstacle parks or stays stationary in a doorway or corridor for **longer than 4.5 seconds** (configurable via `yield_timeout`):
  1. The AMR marks the blocked corridor edge in the topological graph with an impassable cost ($999.0$).
  2. It immediately runs Dijkstra's algorithm to compute an alternate route through a parallel aisle or around the other side of the warehouse.
  3. It publishes the new path to `/agv_dense_path` and smoothly navigates through the detour.
  4. The edge is automatically restored after a cooldown period so future trips can reuse the corridor once cleared.

---

## 3. Adding Your Own Custom Dynamic Obstacles

### Method A: Spawning via Launch Argument
You can spawn dynamic obstacles at any warehouse coordinates with custom speeds:
```bash
ros2 launch agv_description dynamic_obstacle.launch.py \
    name:=worker_cart_2 \
    x:=-2.0 \
    y:=1.5 \
    pattern:=corridor_walker \
    speed:=0.45
```

### Method B: Adding Permanent Moving Actors to `warehouse.world`
If you want dynamic obstacles to always be present when Gazebo opens, open `src/agv_description/worlds/warehouse.world` and add an animated `<actor>` inside `<world name="warehouse">`:

```xml
<actor name="moving_warehouse_worker">
  <skin>
    <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
  </skin>
  <animation name="walk">
    <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
  </animation>
  <script>
    <loop>true</loop>
    <auto_start>true</auto_start>
    <trajectory id="0" type="walk">
      <!-- Waypoint 1: Start at Aisle 1 -->
      <waypoint>
        <time>0.0</time>
        <pose>0 -2.0 1.0 0 0 1.57</pose>
      </waypoint>
      <!-- Waypoint 2: Walk to North End -->
      <waypoint>
        <time>8.0</time>
        <pose>0 2.0 1.0 0 0 1.57</pose>
      </waypoint>
      <!-- Waypoint 3: Turn Around and Walk Back -->
      <waypoint>
        <time>16.0</time>
        <pose>0 -2.0 1.0 0 0 -1.57</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

### Method C: Customizing Obstacle Size and Color
The obstacle model is located in:
[`src/agv_description/models/dynamic_obstacle/model.sdf`](file:///home/rahul/AMR/AMR-main/src/agv_description/models/dynamic_obstacle/model.sdf)

- **To adjust size**: Edit `<radius>` and `<length>` in the `<geometry><cylinder>` tag.
- **To change color**: Edit `<ambient>` and `<diffuse>` in the `<material>` tag (default is high-visibility bright red `0.95 0.15 0.1 1`).
