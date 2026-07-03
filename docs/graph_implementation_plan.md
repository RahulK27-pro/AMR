# Automatic Topological Graph Generation (Adaptive Grid Approach)

We will develop a Python script `graph_extractor.py` that processes a SLAM-generated `.pgm` and `.yaml` map pair and outputs a connected topological navigation graph in JSON format (`warehouse_graph.json`). 

Following the revised requirements, this pipeline will abandon skeletonization as the primary graph structure. Instead, it will use a distance transform and an adaptive grid sampling technique to ensure complete coverage of both narrow corridors and open rooms, while maintaining safe clearances from obstacles.

## User Review Required

> [!IMPORTANT]
> The target map files identified are `warehouse_map.pgm` and `warehouse_map.yaml` located in `/home/rahul/AMR/AMR-main/src/agv_description/maps/`.
> The output `warehouse_graph.json` will be saved in the same `maps` directory.
> I will write `graph_extractor.py` into `/home/rahul/AMR/AMR-main/src/agv_description/scripts/` (or `maps/`). Please confirm if this is acceptable.

## Proposed Changes

We will create a new Python script utilizing `opencv-python` (cv2), `numpy`, `networkx`, and `pyyaml`.

### `agv_description/maps/`

#### [NEW] [graph_extractor.py](file:///\\wsl.localhost\Ubuntu\home\rahul\AMR\AMR-main\src\agv_description\maps\graph_extractor.py)
This script will implement the following pipeline:
1. **Load Map:** 
   - Parse `warehouse_map.yaml` to obtain `resolution` and `origin`.
   - Read `warehouse_map.pgm` using OpenCV in grayscale.
2. **Binary Occupancy:** 
   - Convert map to a binary format (0 = occupied, 1/255 = free).
3. **Distance Transform:**
   - Compute the distance to the nearest wall for every free pixel using `cv2.distanceTransform`. 
   - Convert these distances from pixels to meters using the map resolution.
4. **Adaptive Grid Sampling:**
   - We will sample candidate nodes across the bounding box of the free space. 
   - Instead of a fixed grid, we will evaluate the local clearance (from the distance transform) at grid generation to adapt the spacing:
     - **Narrow corridors (clearance < 1.0 m):** ~40 cm spacing
     - **Normal aisles (1.0 m <= clearance <= 2.0 m):** ~50 cm spacing
     - **Large open rooms (clearance > 2.0 m):** ~80 cm spacing
   - *Implementation Note:* A practical way to achieve this is to generate a dense grid (e.g., 40cm spacing), and selectively drop candidate nodes in high-clearance areas to achieve the desired sparsity, ensuring nodes don't cluster unnecessarily in open rooms.
5. **Safety Rejection:**
   - Reject any candidate node where the distance transform indicates a clearance of less than 25 cm (Robot radius 15cm + Safety margin 10cm).
6. **Graph Connectivity (Line of Sight):**
   - For every valid node, search for neighbors using a radius search (e.g., within 1.5x the local grid spacing).
   - Perform a Bresenham line check between node pairs on the inflated obstacle map (or check if any pixel on the line has clearance < 25cm). 
   - Only create an edge if the line of sight is entirely collision-free.
7. **Graph Optimization:**
   - Prune isolated nodes (degree == 0).
   - Run a connected components check. If multiple isolated graphs exist, keep only the largest one (assuming a single contiguous traversable warehouse) or log a warning.
8. **Export & Visualization:** 
   - Serialize the `networkx` graph to `warehouse_graph.json` containing nodes, real-world coordinates, and edge costs.
   - Output debug images (e.g., overlaying the adaptive grid and edges on top of the original map) so you can visually verify the arrangement.

## Verification Plan

### Automated Tests
- Execute `graph_extractor.py` against `warehouse_map.pgm`.

### Manual Verification
- Review the generated visualization image to ensure grid spacing adapts correctly in open rooms vs narrow corridors.
- Verify that no nodes are closer than 25 cm to any wall.
- Verify `warehouse_graph.json` contains sensible node coordinates and edge costs matching the map's scale.
