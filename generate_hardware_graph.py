#!/usr/bin/env python3
"""
generate_hardware_graph.py
==========================
Converts any physical occupancy grid map into a topological navigation roadmap (nodes & edges)
using the project's Distance-Transform Adaptive Multi-Resolution Roadmap algorithm.

Usage:
  python3 generate_hardware_graph.py [MAP_NAME_OR_PATH]

Examples:
  python3 generate_hardware_graph.py prakash_room
  python3 generate_hardware_graph.py final_map
  python3 generate_hardware_graph.py maps/my_clean_warehouse11_map.yaml
"""

import os
import sys
from pathlib import Path

ws_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ws_dir, "src", "agv_description", "maps"))
from graph_extractor import extract_graph


def main():
    map_input = sys.argv[1] if len(sys.argv) > 1 else "prakash_room"

    ws_dir = os.path.dirname(os.path.abspath(__file__))
    ws_maps = os.path.join(ws_dir, "maps")
    pkg_maps = os.path.join(ws_dir, "src", "agv_description", "maps")

    # 1. Resolve map YAML file
    if os.path.isabs(map_input) and os.path.exists(map_input):
        yaml_path = map_input
    else:
        clean = map_input[:-5] if map_input.endswith('.yaml') else map_input
        candidates = [
            os.path.join(ws_maps, f"{clean}.yaml"),
            os.path.join(pkg_maps, f"{clean}.yaml"),
            os.path.join(ws_dir, f"{clean}.yaml"),
        ]
        yaml_path = None
        for c in candidates:
            if os.path.exists(c):
                yaml_path = c
                break

    if not yaml_path or not os.path.exists(yaml_path):
        print(f"[ERROR] Could not find map YAML matching '{map_input}'.")
        print(f"Checked:\n  - {os.path.join(ws_maps, f'{map_input}.yaml')}\n  - {os.path.join(pkg_maps, f'{map_input}.yaml')}")
        sys.exit(1)

    stem = Path(yaml_path).stem
    out_dir = os.path.dirname(yaml_path)
    output_json = os.path.join(out_dir, f"{stem}_graph.json")
    output_vis = os.path.join(out_dir, f"{stem}_graph_vis.png")

    print("==================================================================")
    print("  AMR Topological Roadmap Node Extractor")
    print(f"  Input Map YAML  : {yaml_path}")
    print(f"  Target JSON     : {output_json}")
    print(f"  Target Preview  : {output_vis}")
    print("==================================================================")

    # 2. Extract topological graph using the calibrated parameters
    # Robot radius: 0.11m (22cm chassis diameter), Safety margin: 0.09m (Total 0.20m clearance)
    graph_data = extract_graph(
        map_yaml_path=yaml_path,
        robot_radius=0.11,
        safety_margin=0.09,
        global_search_radius=2.5,
        step_corridor=0.40,
        step_medium=0.55,
        step_open=0.85,
        output_json=output_json,
        output_vis=output_vis
    )

    # Also sync a copy to src/agv_description/maps if different
    if out_dir != pkg_maps and os.path.exists(pkg_maps):
        sync_json = os.path.join(pkg_maps, f"{stem}_graph.json")
        sync_vis = os.path.join(pkg_maps, f"{stem}_graph_vis.png")
        try:
            import shutil
            shutil.copy2(output_json, sync_json)
            if os.path.exists(output_vis):
                shutil.copy2(output_vis, sync_vis)
            print(f"[OK] Synced copy to package maps directory: {sync_json}")
        except Exception:
            pass

    nodes = graph_data.get("nodes", [])
    edges = graph_data.get("edges", [])

    print("")
    print("==================================================================")
    print(f"  ✅ Extraction Complete for '{stem}'!")
    print(f"  Total Roadmap Nodes : {len(nodes)}")
    print(f"  Total Valid Edges   : {len(edges)}")
    print(f"  JSON Graph File     : {output_json}")
    print(f"  Visual Image File   : {output_vis}")
    print("==================================================================")
    print(f"  You can now start navigation with:")
    print(f"    ./start_hardware_nav.sh 10.244.248.208 {stem}")
    print("==================================================================")


if __name__ == '__main__':
    main()
