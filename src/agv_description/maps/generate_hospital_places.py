#!/usr/bin/env python3
import json
import math
import shutil
from pathlib import Path

maps_dir = Path(__file__).resolve().parent

places_spec = [
    {
        "id": "place_reception",
        "name": "Reception & Waiting Area",
        "target_x": -1.95,
        "target_y": -3.00,
        "type": "reception",
        "icon": "🏥",
        "color": "#00d4ff",
        "description": "Main hospital lobby, patient check-in counter, and visitor seating"
    },
    {
        "id": "place_r1",
        "name": "Patient Room 1 (R1)",
        "target_x": -6.00,
        "target_y": 3.50,
        "type": "patient_room",
        "icon": "🛏️",
        "color": "#10b981",
        "description": "Patient care room with bed, bedside monitor, and equipment cart"
    },
    {
        "id": "place_r2",
        "name": "Patient Room 2 (R2)",
        "target_x": -3.20,
        "target_y": 3.50,
        "type": "patient_room",
        "icon": "🛏️",
        "color": "#10b981",
        "description": "Patient care room with bed, bedside monitor, and visitor chair"
    },
    {
        "id": "place_dr",
        "name": "Doctor's Consultation Room",
        "target_x": -0.30,
        "target_y": 4.50,
        "type": "doctor_room",
        "icon": "👨‍⚕️",
        "color": "#3b82f6",
        "description": "Physician desk, diagnostic monitor, and patient examination couch"
    },
    {
        "id": "place_lab",
        "name": "Clinical Laboratory",
        "target_x": 3.00,
        "target_y": 3.50,
        "type": "laboratory",
        "icon": "🔬",
        "color": "#a855f7",
        "description": "Diagnostic analyzers, centrifuge benches, and reagent storage"
    },
    {
        "id": "place_sc",
        "name": "Sample Collection Room",
        "target_x": 6.00,
        "target_y": 3.50,
        "type": "sample_collection",
        "icon": "🧪",
        "color": "#f59e0b",
        "description": "Blood drawing phlebotomy station and specimen intake counter"
    },
    {
        "id": "place_ph",
        "name": "Hospital Pharmacy",
        "target_x": -6.00,
        "target_y": -3.50,
        "type": "pharmacy",
        "icon": "💊",
        "color": "#ec4899",
        "description": "Prescription dispensing counter, medication shelving, and safe"
    },
    {
        "id": "place_ns",
        "name": "Nurses Station",
        "target_x": 2.00,
        "target_y": -3.50,
        "type": "nurses_station",
        "icon": "👩‍⚕️",
        "color": "#06b6d4",
        "description": "Central nursing counter, task terminals, and charts cabinet"
    },
    {
        "id": "place_ut",
        "name": "Utility & Sterilization",
        "target_x": 5.00,
        "target_y": -3.50,
        "type": "utility",
        "icon": "🧹",
        "color": "#64748b",
        "description": "Clean linens storage, sterilization racks, and medical cart supply"
    },
    {
        "id": "place_toilet_m",
        "name": "Restroom (M)",
        "target_x": 7.15,
        "target_y": -2.20,
        "type": "restroom",
        "icon": "🚻",
        "color": "#6366f1",
        "description": "Men's restroom facility with vanity sink and fixture"
    },
    {
        "id": "place_toilet_f",
        "name": "Restroom (F)",
        "target_x": 7.00,
        "target_y": -4.80,
        "type": "restroom",
        "icon": "🚻",
        "color": "#6366f1",
        "description": "Women's restroom facility accessible via utility corridor"
    },
    {
        "id": "place_entrance",
        "name": "Hospital Main Entrance",
        "target_x": -1.95,
        "target_y": -5.20,
        "type": "entrance",
        "icon": "🚪",
        "color": "#00ff9d",
        "description": "Exterior double-door entrance from hospital driveway and dropoff"
    },
    {
        "id": "place_corridor_center",
        "name": "Corridor Central Spine",
        "target_x": 0.00,
        "target_y": 0.00,
        "type": "corridor",
        "icon": "🛣️",
        "color": "#00d4ff",
        "description": "Central 2.0m transit corridor connecting East and West hospital wings"
    }
]

for map_name in ['hospital_map']:
    graph_file = maps_dir / f"{map_name}_graph.json"
    if not graph_file.exists():
        continue

    with open(graph_file, 'r', encoding='utf-8') as f:
        graph_data = json.load(f)

    nodes = graph_data.get('nodes', [])
    places = []

    for spec in places_spec:
        tx, ty = spec['target_x'], spec['target_y']
        best_node = None
        min_dist = float('inf')
        for n in nodes:
            d = math.hypot(n['x'] - tx, n['y'] - ty)
            if d < min_dist:
                min_dist = d
                best_node = n

        if best_node:
            places.append({
                "id": spec["id"],
                "name": spec["name"],
                "node_id": best_node["id"],
                "type": spec["type"],
                "x": round(best_node["x"], 3),
                "y": round(best_node["y"], 3),
                "px": best_node.get("px", 0),
                "py": best_node.get("py", 0),
                "theta": 0.0,
                "icon": spec["icon"],
                "color": spec["color"],
                "description": spec["description"]
            })

    # Save dedicated places file
    places_file = maps_dir / f"{map_name}_places.json"
    with open(places_file, 'w', encoding='utf-8') as f:
        json.dump(places, f, indent=2)
    print(f"Saved {len(places)} named places to {places_file}")

    # Inject into graph file
    graph_data['places'] = places
    with open(graph_file, 'w', encoding='utf-8') as f:
        json.dump(graph_data, f, indent=2)
    print(f"Injected places into {graph_file}")

    # Mirror to app/src/agv_description/maps
    app_maps_dir = maps_dir.parents[2] / "app" / "src" / "agv_description" / "maps"
    if app_maps_dir.exists():
        shutil.copy2(str(places_file), str(app_maps_dir / f"{map_name}_places.json"))
        shutil.copy2(str(graph_file), str(app_maps_dir / f"{map_name}_graph.json"))
        print(f"Mirrored places & graph to {app_maps_dir}")

print("Places generation complete!")
