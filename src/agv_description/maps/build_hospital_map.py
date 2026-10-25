#!/usr/bin/env python3
"""
build_hospital_map.py
=====================
Generates a geometrically exact, high-precision occupancy grid map (PGM, YAML, PNG)
for the hospital environment, directly derived from src/agv_description/worlds/hospital.world.

This completely eliminates SLAM sensor noise, drift, collision smudges, and blurry walls,
providing a clean clinical floorplan with open doorways and obstacles ready for graph extraction.
"""

import os
import math
import yaml
import numpy as np
from PIL import Image, ImageDraw
import xml.etree.ElementTree as ET
from pathlib import Path

def generate_hospital_map(ws_root: Path, output_names=['hospital_map']):
    world_path = ws_root / "src" / "agv_description" / "worlds" / "hospital.world"
    maps_dir = ws_root / "src" / "agv_description" / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)

    print(f"Reading world specification from {world_path}...")
    tree = ET.parse(str(world_path))
    root = tree.getroot()
    world_el = root.find('world')
    if world_el is None:
        raise ValueError("No <world> element found in hospital.world")

    # Map grid configuration
    # Hospital building is 16m x 12m: X in [-8.0, 8.0], Y in [-6.0, 6.0]
    # Add 0.5m perimeter padding around exterior walls
    origin_x = -8.5
    origin_y = -6.5
    res = 0.05  # 5cm per pixel
    width_m = 17.0
    height_m = 13.0

    width = int(round(width_m / res))    # 340 px
    height = int(round(height_m / res))  # 260 px

    print(f"Grid size: {width} x {height} px ({width_m}m x {height_m}m @ {res}m/px), origin: [{origin_x}, {origin_y}, 0]")

    # Start with all UNKNOWN (205 in ROS standard trinary)
    grid = np.full((height, width), 205, dtype=np.uint8)

    # Helper coordinate transforms
    def world_to_grid(wx, wy):
        gx = int(round((wx - origin_x) / res))
        gy = int(round((wy - origin_y) / res))
        return gx, gy

    # Set interior building footprint to FREE SPACE (254)
    # Interior bounds: X in [-7.85, 7.85], Y in [-5.85, 5.85]
    gx_min, gy_min = world_to_grid(-7.85, -5.85)
    gx_max, gy_max = world_to_grid(7.85, 5.85)
    grid[gy_min:gy_max+1, gx_min:gx_max+1] = 254

    # Extract all collision obstacles from hospital.world
    models = world_el.findall('model')
    obstacles = []
    
    for m in models:
        m_name = m.attrib.get('name', '')
        if m_name in ('hospital_floor', 'ground_plane'):
            continue
        link = m.find('link')
        if link is None:
            continue
        col = link.find('collision')
        if col is None:
            continue
        geom = col.find('geometry')
        if geom is None:
            continue

        pose_el = m.find('pose')
        pose_str = pose_el.text.strip() if pose_el is not None else '0 0 0 0 0 0'
        pose_vals = [float(x) for x in pose_str.split()]
        x, y, z = pose_vals[0], pose_vals[1], pose_vals[2]
        yaw = pose_vals[5] if len(pose_vals) > 5 else 0.0

        box = geom.find('box')
        cyl = geom.find('cylinder')

        if box is not None:
            size_vals = [float(v) for v in box.find('size').text.strip().split()]
            sx, sy, sz = size_vals[0], size_vals[1], size_vals[2]
            # Verify obstacle intersects LiDAR plane (z around 0.155m)
            if (z - sz/2.0) <= 0.30 and (z + sz/2.0) >= 0.10:
                obstacles.append({
                    'name': m_name,
                    'type': 'box',
                    'x': x, 'y': y,
                    'sx': sx, 'sy': sy,
                    'yaw': yaw
                })
        elif cyl is not None:
            r = float(cyl.find('radius').text.strip())
            l = float(cyl.find('length').text.strip())
            if (z - l/2.0) <= 0.30 and (z + l/2.0) >= 0.10:
                obstacles.append({
                    'name': m_name,
                    'type': 'cylinder',
                    'x': x, 'y': y,
                    'r': r,
                    'yaw': yaw
                })

    print(f"Rasterizing {len(obstacles)} collision models onto occupancy grid...")

    # Create PIL image for rasterizing solid occupied obstacles (0 = occupied)
    # Note: in PIL, (0,0) is top-left, corresponding to gy = height - 1
    im_grid = Image.fromarray(grid)
    draw = ImageDraw.Draw(im_grid)

    def world_to_pil(wx, wy):
        gx = (wx - origin_x) / res
        gy = (wy - origin_y) / res
        py = (height - 1) - gy
        px = gx
        return px, py

    for obs in obstacles:
        ox, oy = obs['x'], obs['y']
        yaw = obs.get('yaw', 0.0)

        if obs['type'] == 'box':
            sx, sy = obs['sx'], obs['sy']
            # Compute 4 box corners in local frame
            dx = sx / 2.0
            dy = sy / 2.0
            local_corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
            
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            
            poly_pts = []
            for lx, ly in local_corners:
                wx = ox + (lx * cos_y - ly * sin_y)
                wy = oy + (lx * sin_y + ly * cos_y)
                px, py = world_to_pil(wx, wy)
                poly_pts.append((px, py))
            
            draw.polygon(poly_pts, fill=0, outline=0)

        elif obs['type'] == 'cylinder':
            r = obs['r']
            px, py = world_to_pil(ox, oy)
            r_px = r / res
            draw.ellipse([px - r_px, py - r_px, px + r_px, py + r_px], fill=0, outline=0)

    # Double check outer perimeter walls are solidly occupied (0)
    # North exterior wall
    p1 = world_to_pil(-8.0, 5.85)
    p2 = world_to_pil(8.0, 6.0)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    # West exterior wall
    p1 = world_to_pil(-8.0, -6.0)
    p2 = world_to_pil(-7.85, 6.0)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    # East exterior wall
    p1 = world_to_pil(7.85, -6.0)
    p2 = world_to_pil(8.0, 6.0)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    # South exterior wall - West segment: [-8.0, -2.55]
    p1 = world_to_pil(-8.0, -6.0)
    p2 = world_to_pil(-2.55, -5.85)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    # South exterior wall - East segment: [-1.35, 8.0]
    p1 = world_to_pil(-1.35, -6.0)
    p2 = world_to_pil(8.0, -5.85)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    # Clear doorway openings (ensure doorways are clean free space = 254)
    doorways = [
        # North Corridor Wall doorways (Y from +1.00 to +1.15)
        ('Door R1', -5.75, -4.85, 0.98, 1.17),
        ('Door R2', -2.55, -1.65, 0.98, 1.17),
        ('Door DR', -1.45, -0.55, 0.98, 1.17),
        ('Door LAB', 1.70, 2.60, 0.98, 1.17),
        ('Door SC',  4.85, 5.75, 0.98, 1.17),
        # South Corridor Wall doorways (Y from -1.15 to -1.00)
        ('Door PH', -5.55, -4.65, -1.17, -0.98),
        ('Passage RW', -2.75, -1.15, -1.17, -0.98),
        ('Door NS', 0.75, 1.65, -1.17, -0.98),
        ('Door UT', 3.90, 4.80, -1.17, -0.98),
        ('Door Toilet M', 6.50, 7.40, -1.17, -0.98),
        # Interior Toilets doorway (Toilet F)
        ('Door Toilet F', 6.15, 6.80, -4.70, -4.15),
        # Main South Entrance
        ('Main Entrance', -2.55, -1.35, -6.02, -5.83),
    ]

    for d_name, x0, x1, y0, y1 in doorways:
        p1 = world_to_pil(x0, y0)
        p2 = world_to_pil(x1, y1)
        draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=254, outline=254)


    final_pgm_arr = np.array(im_grid)

    # Save to disk for each requested output name
    for name in output_names:
        yaml_path = maps_dir / f"{name}.yaml"
        pgm_path = maps_dir / f"{name}.pgm"
        png_path = maps_dir / f"{name}.png"

        # 1. PGM
        im_grid.save(str(pgm_path))
        print(f"Saved: {pgm_path} ({pgm_path.stat().st_size} bytes)")

        # 2. YAML
        yaml_content = (
            f"image: {name}.pgm\n"
            f"mode: trinary\n"
            f"resolution: {res:.4f}\n"
            f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        print(f"Saved: {yaml_path}")

        # 3. Clean High-Tech Dark PNG Preview for Dashboard
        # In PNG preview:
        # 205 (unknown) -> dark slate [26, 32, 46]
        # 254 (free space) -> deep clinical slate [12, 16, 28]
        # 0 (occupied walls/furniture) -> vibrant cyan [0, 212, 255]
        h_arr, w_arr = final_pgm_arr.shape
        rgb_preview = np.zeros((h_arr, w_arr, 3), dtype=np.uint8)
        rgb_preview[final_pgm_arr == 205] = [26, 32, 46]     # unknown
        rgb_preview[final_pgm_arr == 254] = [12, 16, 28]     # free space
        rgb_preview[final_pgm_arr == 0]   = [0, 212, 255]    # walls / obstacles
        
        pil_preview = Image.fromarray(rgb_preview)
        pil_preview.save(str(png_path))
        print(f"Saved: {png_path} ({png_path.stat().st_size} bytes)")

        # Copy to app/src/agv_description/maps if it exists
        app_maps_dir = ws_root / "app" / "src" / "agv_description" / "maps"
        if app_maps_dir.exists():
            import shutil
            shutil.copy2(str(pgm_path), str(app_maps_dir / f"{name}.pgm"))
            shutil.copy2(str(yaml_path), str(app_maps_dir / f"{name}.yaml"))
            shutil.copy2(str(png_path), str(app_maps_dir / f"{name}.png"))
            print(f"Mirrored to: {app_maps_dir / name}")

    print("\nHospital map generation complete!")
    print(f"Total free cells: {np.sum(final_pgm_arr == 254)}")
    print(f"Total occupied cells: {np.sum(final_pgm_arr == 0)}")
    print(f"Total unknown cells: {np.sum(final_pgm_arr == 205)}")

if __name__ == '__main__':
    ws = Path(__file__).resolve().parents[3]
    generate_hospital_map(ws)
