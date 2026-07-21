# Graph Extraction Algorithm Overview

This document details the algorithmic pipeline utilized by `graph_extractor.py` to convert a 2D SLAM occupancy grid into a mathematically optimal, traversable topological node graph for the robot's navigation stack.

## 1. Image Processing & Distance Transform
The process begins by ingesting the SLAM-generated map (a `.pgm` or image file alongside a `.yaml` parameter file).
* **Binarization:** The grayscale map is thresholded. Free space is converted to absolute white (255) and obstacles/unknowns to absolute black (0).
* **Euclidean Distance Transform (EDT):** A critical operation is performed using OpenCV's `distanceTransform`. This assigns every free-space pixel a value representing its exact distance to the nearest black pixel (obstacle). This creates a "clearance map" that the algorithm uses to guarantee the physical robot won't scrape against walls.

## 2. Adaptive Multi-Resolution Grid Sampling
To optimize the number of waypoints (nodes) and prevent the MPPI controller from being overwhelmed, the algorithm intelligently varies the density of the nodes based on how tight the environment is.
* Three separate, uniform grids are virtually overlaid on the map with varying resolutions: `0.4m`, `0.5m`, and `0.8m`.
* **Selection Rule:** 
  * If a coordinate has **High Clearance (>2.0m)** from a wall, it pulls from the sparse `0.8m` grid.
  * If a coordinate has **Medium Clearance (1.0m to 2.0m)**, it pulls from the `0.5m` grid.
  * If a coordinate has **Low Clearance (<1.0m)**, it pulls from the dense `0.4m` grid to provide fine-grained navigation through tight doors and corridors.
* **Safety Pruning:** Any node whose clearance is strictly less than the `min_clearance` (Robot Radius + Safety Margin = 0.25m) is permanently discarded.

## 3. Edge Formation via Bresenham Line-of-Sight
With the candidate nodes placed, the algorithm must connect them.
* **Global Search Radius:** Every node looks at all other nodes within a **2.5-meter radius**. 
* **Bresenham's Line Algorithm:** To check if an edge between two nodes is valid, it traces the discrete pixels forming a straight line between them.
* **Lethal Obstacle Check:** It sweeps along this line, checking the Distance Transform value of *every single pixel* underneath the line. If even one pixel has a clearance lower than `min_clearance` (0.25m), it means the robot's physical body would clip a wall while driving on that line. The edge is instantly rejected.
* **Line-of-Sight Pruning:** Because the search radius is large (2.5m), the algorithm naturally finds long, straight line-of-sight connections across open areas, natively skipping intermediate grid points and preventing "zig-zag" paths.

## 4. Component Analysis & Isolation Pruning
Once all safe edges are established, the graph might contain disjointed clusters (e.g., nodes generated inside a closed-off room).
* **Breadth-First Search (BFS):** A BFS traverse is executed to group nodes into "Connected Components".
* **Pruning:** Any node with 0 edges is deleted. If there are multiple disconnected islands of nodes, the algorithm discards all of them except the largest one. This guarantees that if a path is requested between any two nodes in the final graph, a valid route absolutely exists.

## 5. Bidirectional JSON Export
Finally, the valid nodes and edges are exported to `warehouse_graph.json`.
* Nodes are translated from image pixel coordinates back into real-world ROS coordinates using the SLAM map's `origin` and `resolution` parameters.
* To guarantee the navigation node can traverse segments in either direction, the script explicitly exports two directed edges (`A -> B` and `B -> A`) for every valid connection.
