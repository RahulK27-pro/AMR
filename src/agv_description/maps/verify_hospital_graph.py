#!/usr/bin/env python3
import json
import heapq
import numpy as np
from pathlib import Path

maps_dir = Path(__file__).resolve().parent
graph_path = maps_dir / "hospital_map_graph.json"

with open(graph_path, 'r', encoding='utf-8') as f:
    graph = json.load(f)

nodes = graph['nodes']
edges = graph['edges']
num_nodes = len(nodes)
num_edges = len(edges)

print(f"=== HOSPITAL TOPOLOGICAL ROADMAP AUDIT ===")
print(f"Total Nodes: {num_nodes}")
print(f"Total Directed Edges: {num_edges} ({num_edges // 2} bidirectional edges)")

clearances = [n['clearance'] for n in nodes]
print(f"Min Clearance: {min(clearances):.3f} m")
print(f"Avg Clearance: {np.mean(clearances):.3f} m")
print(f"Max Clearance: {max(clearances):.3f} m")

# Build adjacency list
adj = {n['id']: [] for n in nodes}
node_by_id = {n['id']: n for n in nodes}
for e in edges:
    adj[e['from']].append((e['to'], e['cost']))

# Connected components
visited = set()
components = []
for n in nodes:
    nid = n['id']
    if nid not in visited:
        comp = []
        q = [nid]
        visited.add(nid)
        while q:
            curr = q.pop(0)
            comp.append(curr)
            for neighbor, cost in adj[curr]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    q.append(neighbor)
        components.append(comp)

print(f"Connected Components: {len(components)}")
for idx, c in enumerate(components):
    xs = [node_by_id[nid]['x'] for nid in c]
    ys = [node_by_id[nid]['y'] for nid in c]
    print(f"  Component {idx+1}: {len(c)} nodes, X in [{min(xs):.2f}, {max(xs):.2f}], Y in [{min(ys):.2f}, {max(ys):.2f}]")

# Verify nodes inside each room
room_bounds = {
    'R1 (Patient Room 1)':       (-7.85, -4.725,  1.15, 5.85),
    'R2 (Patient Room 2)':       (-4.725, -1.575, 1.15, 5.85),
    'DR (Doctor Room)':          (-1.575,  1.575, 1.15, 5.85),
    'LAB (Laboratory)':          ( 1.575,  4.725, 1.15, 5.85),
    'SC (Sample Collection)':    ( 4.725,  7.85,  1.15, 5.85),
    'PH (Pharmacy)':             (-7.85, -4.525, -5.85, -1.15),
    'RW (Reception & Waiting)':  (-4.525,  0.625, -5.85, -1.15),
    'NS (Nurses Station)':       ( 0.625,  3.775, -5.85, -1.15),
    'UT (Utility)':              ( 3.775,  6.325, -5.85, -1.15),
    'Toilet M (Men)':            ( 6.325,  7.85,  -3.50, -1.15),
    'Toilet F (Women)':          ( 6.325,  7.85,  -5.85, -3.50),
    'Corridor Spine':            (-7.70,  7.70,   -0.95,  0.95),
}

room_nodes = {r: [] for r in room_bounds}
for n in nodes:
    x, y = n['x'], n['y']
    for r, (x0, x1, y0, y1) in room_bounds.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            room_nodes[r].append(n['id'])
            break

print(f"\n{'Room Name':26s} | {'Node Count':10s} | {'Sample Node ID':15s} | {'Position (X, Y)'}")
print("-" * 75)
for r, r_nids in room_nodes.items():
    if r_nids:
        sample_node = node_by_id[r_nids[len(r_nids)//2]]
        print(f"{r:26s} | {len(r_nids):10d} | {sample_node['id']:15s} | ({sample_node['x']:+5.2f}, {sample_node['y']:+5.2f})")
    else:
        print(f"{r:26s} | {0:10d} | {'NONE':15s} | -")

# Run Dijkstra routing probes between all rooms and the Reception / Main Entrance
def dijkstra(start_id, goal_id):
    dist = {nid: float('inf') for nid in adj}
    dist[start_id] = 0
    pq = [(0, start_id)]
    prev = {}
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal_id:
            path = []
            curr = u
            while curr in prev:
                path.append(curr)
                curr = prev[curr]
            path.append(start_id)
            return True, d, list(reversed(path))
        if d > dist[u]:
            continue
        for v, w in adj[u]:
            if d + w < dist[v]:
                dist[v] = d + w
                prev[v] = u
                heapq.heappush(pq, (dist[v], v))
    return False, float('inf'), []

print("\n=== INTER-ROOM ROUTE AUDIT (Dijkstra from Reception) ===")
reception_node = room_nodes['RW (Reception & Waiting)'][0]
all_passed = True
for r, r_nids in room_nodes.items():
    if r == 'RW (Reception & Waiting)':
        continue
    target_node = r_nids[0]
    ok, distance, path = dijkstra(reception_node, target_node)
    status = "OK" if ok else "FAILED"
    if not ok:
        all_passed = False
    print(f"Route to {r:26s}: [{status}] Distance: {distance:5.2f}m, Hops: {len(path)-1:3d}")

print(f"\nFinal Audit Verdict: {'ALL CHECKS PASSED PERFECTLY (100% Traversable)' if all_passed else 'SOME ROUTES FAILED'}")
