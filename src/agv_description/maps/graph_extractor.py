import cv2
import numpy as np
import yaml
import json
import os
import argparse

def bresenham_line(x0, y0, x1, y1):
    """Bresenham's Line Algorithm. Produces a list of tuples from start to end."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x, y))
    return points

def extract_graph(map_yaml_path):
    print(f"Loading map from {map_yaml_path}...")
    with open(map_yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)
    
    resolution = map_info['resolution']
    origin = map_info['origin']
    image_name = map_info['image']
    
    dir_name = os.path.dirname(map_yaml_path)
    if not dir_name:
        dir_name = '.'
    image_path = os.path.join(dir_name, image_name)
    
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Could not read image: {image_path}")
    
    height, width = img.shape
    
    # Thresholding: 255 is free space, 0 is occupied
    _, binary = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
    
    # Distance transform
    print("Computing Distance Transform...")
    dist_transform = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
    
    robot_radius = 0.15
    safety_margin = 0.10
    min_clearance = robot_radius + safety_margin  # 0.25m
    
    print("Generating Adaptive Grid...")
    # Generate multi-resolution candidate grids
    nodes_40 = []
    nodes_50 = []
    nodes_80 = []
    
    step_40 = max(1, int(0.40 / resolution))
    step_50 = max(1, int(0.50 / resolution))
    step_80 = max(1, int(0.80 / resolution))
    
    for y in range(0, height, step_40):
        for x in range(0, width, step_40):
            nodes_40.append((x, y))
            
    for y in range(0, height, step_50):
        for x in range(0, width, step_50):
            nodes_50.append((x, y))
            
    for y in range(0, height, step_80):
        for x in range(0, width, step_80):
            nodes_80.append((x, y))
            
    candidate_nodes = []
    
    # Sample nodes based on local clearance
    for (x, y) in nodes_80:
        c = dist_transform[y, x] * resolution
        if c > 2.0:
            candidate_nodes.append((x, y))
            
    for (x, y) in nodes_50:
        c = dist_transform[y, x] * resolution
        if 1.0 <= c <= 2.0:
            candidate_nodes.append((x, y))
            
    for (x, y) in nodes_40:
        c = dist_transform[y, x] * resolution
        if min_clearance <= c < 1.0:
            candidate_nodes.append((x, y))
            
    print(f"Generated {len(candidate_nodes)} candidate nodes.")
    
    # Build Graph
    print("Connecting nodes using Line of Sight checks...")
    edges = []
    adj = {i: [] for i in range(len(candidate_nodes))}
    
    for i in range(len(candidate_nodes)):
        x0, y0 = candidate_nodes[i]
        c0 = dist_transform[y0, x0] * resolution
        
        # Global uniform search radius to prevent asymmetric edge breaks
        # Set to 2.5m to natively find long line-of-sight connections, straightening zig-zags!
        global_search_radius = 2.5 
        radius_px = global_search_radius / resolution
        
        for j in range(i + 1, len(candidate_nodes)):
            x1, y1 = candidate_nodes[j]
            dist_px = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
            
            if dist_px <= radius_px:
                # Line of sight check
                line = bresenham_line(x0, y0, x1, y1)
                collision = False
                for (lx, ly) in line:
                    if lx < 0 or lx >= width or ly < 0 or ly >= height:
                        collision = True
                        break
                    # We check if line goes through obstacles (dist < min_clearance)
                    if dist_transform[ly, lx] * resolution < min_clearance:
                        collision = True
                        break
                
                if not collision:
                    edges.append((i, j, dist_px * resolution))
                    adj[i].append(j)
                    adj[j].append(i)
                    
    # Prune isolated nodes
    isolated = [i for i in range(len(candidate_nodes)) if len(adj[i]) == 0]
    valid_nodes = set(range(len(candidate_nodes))) - set(isolated)
    print(f"Pruned {len(isolated)} isolated nodes.")
    
    # Keep largest connected component
    visited = set()
    components = []
    
    for i in valid_nodes:
        if i not in visited:
            comp = set()
            queue = [i]
            while queue:
                curr = queue.pop(0)
                if curr not in visited:
                    visited.add(curr)
                    comp.add(curr)
                    for neighbor in adj[curr]:
                        if neighbor not in visited:
                            queue.append(neighbor)
            components.append(comp)
            
    if len(components) > 1:
        print(f"Warning: Found {len(components)} connected components. Keeping the largest one.")
        largest_cc = max(components, key=len)
        valid_nodes = largest_cc
        
    print(f"Final graph has {len(valid_nodes)} nodes.")
    
    # Relabel nodes to be sequential
    old_to_new = {}
    new_idx = 0
    for old_idx in sorted(list(valid_nodes)):
        old_to_new[old_idx] = new_idx
        new_idx += 1
        
    # Export to JSON
    json_graph = {
        "nodes": [],
        "edges": []
    }
    
    for old_idx in sorted(list(valid_nodes)):
        n = old_to_new[old_idx]
        px, py = candidate_nodes[old_idx]
        # Convert to ROS world coordinates (origin is bottom-left usually)
        wx = origin[0] + px * resolution
        wy = origin[1] + (height - py - 1) * resolution
        
        json_graph["nodes"].append({
            "id": f"N{n}",
            "x": round(wx, 4),
            "y": round(wy, 4),
            "px": px,
            "py": py
        })
        
    exported_edges = set()
    
    for u, v, weight in edges:
        if u in valid_nodes and v in valid_nodes:
            new_u = old_to_new[u]
            new_v = old_to_new[v]
            
            # Forward edge
            edge_fwd = (f"N{new_u}", f"N{new_v}")
            if edge_fwd not in exported_edges:
                json_graph["edges"].append({
                    "from": edge_fwd[0],
                    "to": edge_fwd[1],
                    "cost": round(weight, 4)
                })
                exported_edges.add(edge_fwd)
                
            # Reverse edge
            edge_rev = (f"N{new_v}", f"N{new_u}")
            if edge_rev not in exported_edges:
                json_graph["edges"].append({
                    "from": edge_rev[0],
                    "to": edge_rev[1],
                    "cost": round(weight, 4)
                })
                exported_edges.add(edge_rev)
        
    json_path = os.path.join(dir_name, "warehouse_graph.json")
    with open(json_path, 'w') as f:
        json.dump(json_graph, f, indent=4)
    print(f"Graph exported to {json_path}")
    
    # Visualization
    vis_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    # Draw edges
    for u, v, weight in edges:
        if u in valid_nodes and v in valid_nodes:
            pt1 = (candidate_nodes[u][0], candidate_nodes[u][1])
            pt2 = (candidate_nodes[v][0], candidate_nodes[v][1])
            cv2.line(vis_img, pt1, pt2, (255, 0, 0), 1)
        
    # Draw nodes
    for u in valid_nodes:
        pt = (candidate_nodes[u][0], candidate_nodes[u][1])
        cv2.circle(vis_img, pt, 2, (0, 0, 255), -1)
        
    vis_path = os.path.join(dir_name, "graph_visualization.png")
    cv2.imwrite(vis_path, vis_img)
    print(f"Visualization saved to {vis_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract Topological Graph from Occupancy Grid.")
    parser.add_argument("map_yaml", help="Path to the map .yaml file")
    args = parser.parse_args()
    
    extract_graph(args.map_yaml)
