# Step 2 Implementation Plan — Custom Frontier Scorer

## What We Are Changing

We modify 3 files inside `src/m-explore-ros2/explore/`:

### `include/explore/frontier_search.h`
- Extend constructor to accept `wall_bonus_scale`, `momentum_scale`, and `robot_heading`
- Add two private helper methods: `wallProximityBonus()` and `momentumBonus()`
- Add private members for the new scales and robot heading

### `src/frontier_search.cpp`  
- Implement `wallProximityBonus()`: scans costmap cells near the frontier centroid,
  returns a bonus [0.0–1.0] based on how many adjacent cells are LETHAL_OBSTACLE.
- Implement `momentumBonus()`: computes the dot product between the robot's heading
  vector and the vector from robot to frontier centroid. Returns [0.0–1.0].
- Modify `frontierCost()` to subtract both bonuses.

### `src/explore.cpp`
- Declare two new ROS 2 parameters: `wall_bonus_scale`, `momentum_scale`
- Extract robot heading from the existing TF lookup (already done for `getRobotPose`)
- Pass heading and scales to `FrontierSearch` constructor

### `config/nav2_params_explore.yaml`
- Add `wall_bonus_scale: 2.0` and `momentum_scale: 1.5`

## New Cost Function

```
cost = (potential_scale × distance)
     − (gain_scale × frontier_size)
     − (wall_bonus_scale × wall_bonus)       ← NEW
     − (momentum_scale × momentum_bonus)     ← NEW
```

- `wall_bonus`    ∈ [0.0, 1.0]: proportion of cells near centroid that are walls
- `momentum_bonus` ∈ [0.0, 1.0]: cosine similarity between heading and frontier direction
