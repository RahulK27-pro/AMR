# Phase 2: Topological Graph Extraction

## Overview

In **Phase 2**, the objective is to convert a static occupancy grid (generated offline via SLAM in Phase 1) into a lightweight, navigable topological graph. This graph serves as the road network for the global planner (Dijkstra/A*), providing a highly efficient way to compute paths across large spaces before handing them over to the local controller (e.g., MPPI).

Instead of relying solely on traditional skeletonization (which works well in narrow corridors but struggles in large open rooms by collapsing them into a single path), we implemented an **Adaptive Grid approach driven by a Distance Transform**. This ensures comprehensive coverage across the entire map, maintaining safe clearances while providing multiple routing options in open areas.

---

## Pipeline Implementation Details

The extraction pipeline is implemented in the `graph_extractor.py` script. The process follows several sequential stages to convert raw pixels into a connected navigation network.

### 1. Map Ingestion & Binary Conversion
The process begins by parsing the `warehouse_map.yaml` to extract the map's `resolution` (meters per pixel) and origin coordinates. The corresponding `warehouse_map.pgm` image is loaded in grayscale. 

The image is thresholded into a binary format:
- **Free Space:** Pixels > 200 (white/255)
- **Obstacles/Unknown:** Pixels <= 200 (black/0)

### 2. Distance Transform
The most critical step in the pipeline is generating the **Distance Transform**. Using `cv2.distanceTransform()`, we calculate the Euclidean distance from every free-space pixel to the nearest obstacle. 
This provides a "clearance map" where every pixel knows exactly how far it is from a wall. These pixel distances are immediately scaled by the map's `resolution` to convert them into real-world meters.

### 3. Adaptive Grid Generation
To generate candidate nodes, we project a grid over the map. However, a uniform grid is inefficient: a dense grid creates too many nodes in open rooms, while a sparse grid might fail to navigate tight corridors. 

We solve this using an **Adaptive Grid** based on the local clearance (read directly from the distance transform):
- **Narrow Corridors (Clearance < 1.0 m):** Sampled at a dense **40 cm** spacing.
- **Normal Aisles (1.0 m ≤ Clearance ≤ 2.0 m):** Sampled at a medium **50 cm** spacing.
- **Open Rooms (Clearance > 2.0 m):** Sampled sparsely at an **80 cm** spacing.

*Implementation:* We generate three overlapping theoretical grids (40cm, 50cm, and 80cm). For each point in these grids, we check the distance transform. If the local clearance falls into the point's designated tier, the point is added as a candidate node. This naturally limits node clustering in empty warehouses while ensuring fine-grained maneuverability near shelves and walls.

### 4. Safety Clearance Filtering
During grid generation, any node that falls within the robot's inflation radius is immediately rejected. 
- **Robot Radius:** 15 cm
- **Safety Margin:** 10 cm
- **Minimum Clearance:** 25 cm

If a candidate node has a distance transform value of `< 0.25 m`, it is discarded. This completely replaces the need for a costly image-wide inflation operation, as the distance transform natively provides the inflation boundary.

### 5. Line-of-Sight Graph Connectivity
Once the candidate nodes are established, they must be connected to form edges. 
For every node, we define a search radius equal to **1.5× its local grid spacing** (e.g., a node in a 50cm grid searches up to 75cm away).

For every neighboring pair within this radius, we perform a **Line of Sight check** using the Bresenham line algorithm:
1. We trace the straight line between the two nodes pixel-by-pixel.
2. For every pixel on the line, we check the distance transform.
3. If *any* pixel on the line has a clearance `< 25 cm` (meaning the line grazes an obstacle), the edge is rejected.
4. If the line is completely safe, an edge is created, and the Euclidean distance is assigned as the edge weight/cost.

### 6. Graph Optimization & Connectivity Verification
After all valid edges are created, the graph undergoes optimization:
- **Pruning:** Any node with a degree of 0 (no connections) is deleted.
- **Connected Components Check:** We perform a Breadth-First Search (BFS) to find all independent connected sub-graphs. In a perfect map, there should only be one component. If disconnected islands exist (due to SLAM noise or physically unreachable areas), the algorithm identifies them and retains only the **largest connected component**, guaranteeing that every node in the final graph is reachable from any other node.

### 7. Export & Serialization
Finally, the valid nodes and edges are re-indexed sequentially.
The pixel coordinates `(px, py)` are converted back into ROS real-world coordinates `(x, y)` using the standard transform:
- `World X = Origin X + (Pixel X * Resolution)`
- `World Y = Origin Y + ((Image Height - Pixel Y - 1) * Resolution)`

The data is serialized into a lightweight JSON file (`warehouse_graph.json`) consisting of a `nodes` array and an `edges` array. This file is now ready to be consumed by the global planner in Phase 3.

---

## Results
In the provided test map, the adaptive grid generated **943** initial candidate nodes. After applying line-of-sight connectivity, pruning 3 isolated nodes, and filtering down to the largest connected component, the pipeline successfully produced a robust, fully-traversable navigation graph consisting of **756 nodes**.
