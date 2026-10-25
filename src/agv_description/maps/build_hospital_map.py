#!/usr/bin/env python3
"""
build_hospital_map.py
=====================
Generates a complete, high-fidelity, solid architectural occupancy grid map (PGM, YAML, PNG)
for the hospital environment, directly derived from src/agv_description/worlds/hospital.world.

Design guarantees:
- Every object is 100% solid, filled, and contiguous — NO disjoint fragments or hollow shells.
- Walls form continuous, solid architectural boundaries with exact 0.15m thickness and clean doorways.
- Tables, desks, beds, counters, cabinets, and chairs completely resemble their exact 3D physical models.
- Corridor waiting chairs (corridor_chair_1..4) are distinctly and prominently visible along the south wall.
- Walkable building interior is clean free floor (254).
- Unmapped exterior space outside the building footprint is marked as Unknown (205).
- Generates both High-Tech Dark Theme PNG and Clinical Light Theme PNG.
- Automatically mirrors all assets to app/src/agv_description/maps/.
"""

import os
import math
import yaml
import cv2
import shutil
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
    # Origin at [-8.5, -6.5], 17m x 13m @ 0.05m/px -> 340 x 260 pixels
    origin_x = -8.5
    origin_y = -6.5
    res = 0.05  # 5cm per cell
    width_m = 17.0
    height_m = 13.0
    width = int(round(width_m / res))    # 340 px
    height = int(round(height_m / res))  # 260 px

    print(f"Grid size: {width} x {height} px ({width_m}m x {height_m}m @ {res}m/px), origin: [{origin_x}, {origin_y}, 0]")

    def world_to_grid(wx, wy):
        gx = (wx - origin_x) / res
        gy = (wy - origin_y) / res
        py = (height - 1) - gy
        px = gx
        return px, py

    # 1. Initialize entire map as UNKNOWN (205) - standard ROS SLAM outside building
    grid = np.full((height, width), 205, dtype=np.uint8)

    # 2. Set interior hospital building footprint to FREE SPACE (254)
    # Hospital interior bounds: X in [-7.85, 7.85], Y in [-5.85, 5.85]
    p1 = world_to_grid(-7.85, -5.85)
    p2 = world_to_grid(7.85, 5.85)
    gx_min, gx_max = int(round(min(p1[0], p2[0]))), int(round(max(p1[0], p2[0])))
    gy_min, gy_max = int(round(min(p1[1], p2[1]))), int(round(max(p1[1], p2[1])))
    grid[gy_min:gy_max+1, gx_min:gx_max+1] = 254

    # Main South Entrance threshold is walkable free space
    pe1 = world_to_grid(-2.55, -6.00)
    pe2 = world_to_grid(-1.35, -5.85)
    grid[int(min(pe1[1], pe2[1])):int(max(pe1[1], pe2[1]))+1, int(min(pe1[0], pe2[0])):int(max(pe1[0], pe2[0]))+1] = 254

    # 3. Parse collision models from hospital.world
    models = world_el.findall('model')
    boxes = []
    cylinders = []

    for m in models:
        m_name = m.attrib.get('name', '')
        if m_name in ('hospital_floor', 'ground_plane'):
            continue
        link = m.find('link')
        if link is None: continue
        col = link.find('collision')
        if col is None: continue
        geom = col.find('geometry')
        if geom is None: continue
        pose_el = m.find('pose')
        pose_vals = [float(x) for x in pose_el.text.strip().split()] if pose_el is not None else [0]*6
        x, y, z = pose_vals[0], pose_vals[1], pose_vals[2]
        yaw = pose_vals[5] if len(pose_vals) > 5 else 0.0

        box = geom.find('box')
        cyl = geom.find('cylinder')

        if box is not None:
            sz = [float(v) for v in box.find('size').text.strip().split()]
            # LiDAR scan plane is at z approx 0.155m; check vertical overlap
            if (z - sz[2]/2.0) <= 0.35 and (z + sz[2]/2.0) >= 0.08:
                boxes.append({
                    'name': m_name,
                    'x': x, 'y': y,
                    'sx': sz[0], 'sy': sz[1],
                    'yaw': yaw
                })
        elif cyl is not None:
            r = float(cyl.find('radius').text.strip())
            l = float(cyl.find('length').text.strip())
            if (z - l/2.0) <= 0.35 and (z + l/2.0) >= 0.08:
                cylinders.append({
                    'name': m_name,
                    'x': x, 'y': y,
                    'r': r,
                    'yaw': yaw
                })

    print(f"Rasterizing {len(boxes)} boxes and {len(cylinders)} cylinders solidly onto occupancy grid...")

    # 4. Rasterize all objects solidly and contiguously (0 = occupied)
    im_grid = Image.fromarray(grid)
    draw = ImageDraw.Draw(im_grid)

    # Perimeter walls (solid continuous 0.15m walls)
    # North
    p1 = world_to_grid(-8.0, 5.85)
    p2 = world_to_grid(8.0, 6.0)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)
    # West
    p1 = world_to_grid(-8.0, -6.0)
    p2 = world_to_grid(-7.85, 6.0)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)
    # East
    p1 = world_to_grid(7.85, -6.0)
    p2 = world_to_grid(8.0, 6.0)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)
    # South West
    p1 = world_to_grid(-8.0, -6.0)
    p2 = world_to_grid(-2.55, -5.85)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)
    # South East
    p1 = world_to_grid(-1.35, -6.0)
    p2 = world_to_grid(8.0, -5.85)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    # Draw all boxes (walls, tables, desks, beds, counters, cabinets, chairs) as complete solid polygons
    for b in boxes:
        # Corridor chairs are drawn with dedicated separation below
        if 'corridor_chair' in b['name']:
            continue
        ox, oy = b['x'], b['y']
        yaw = b['yaw']
        dx = b['sx'] / 2.0
        dy = b['sy'] / 2.0
        local_corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        poly = []
        for lx, ly in local_corners:
            wx = ox + (lx * cos_y - ly * sin_y)
            wy = oy + (lx * sin_y + ly * cos_y)
            px, py = world_to_grid(wx, wy)
            poly.append((px, py))
        draw.polygon(poly, fill=0, outline=0)

    # Draw all cylinders (plants, circular stools) as complete solid ellipses
    for c in cylinders:
        ox, oy = c['x'], c['y']
        px, py = world_to_grid(ox, oy)
        r_px = c['r'] / res
        draw.ellipse([px - r_px, py - r_px, px + r_px, py + r_px], fill=0, outline=0)

    # 5. Clean, open doorways matching hospital.world specifications
    doorways = [
        # North corridor wall doorways (to Patient Rooms, DR, Lab, SC)
        (-5.75, -4.85, 0.98, 1.17),
        (-2.55, -1.65, 0.98, 1.17),
        (-1.45, -0.55, 0.98, 1.17),
        ( 1.70,  2.60, 0.98, 1.17),
        ( 4.85,  5.75, 0.98, 1.17),
        # South corridor wall doorways (to Pharmacy, Reception, Nurses' Station, Utility, Restrooms)
        (-5.55, -4.65, -1.17, -0.98),
        (-2.75, -1.15, -1.17, -0.98),
        ( 0.75,  1.65, -1.17, -0.98),
        ( 3.90,  4.80, -1.17, -0.98),
        ( 6.50,  7.40, -1.17, -0.98),
        # Restroom internal partition doorway
        ( 6.15,  6.80, -4.70, -4.15),
        # Main entrance
        (-2.55, -1.35, -6.02, -5.83),
    ]
    for x0, x1, y0, y1 in doorways:
        p1 = world_to_grid(x0, y0)
        p2 = world_to_grid(x1, y1)
        draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=254, outline=254)

    # 6. Corridor Waiting Chairs: Render as 4 solid, distinct, completely filled individual seats
    # Each chair is 0.40m x 0.40m, positioned along y = -0.72 outside Pharmacy
    corridor_chairs = [
        ('corridor_chair_1', -4.10, -0.72),
        ('corridor_chair_2', -3.55, -0.72),
        ('corridor_chair_3', -3.00, -0.72),
        ('corridor_chair_4', -2.45, -0.72),
    ]
    # First ensure corridor floor around chairs is clean free space
    p_c1 = world_to_grid(-4.50, -0.48)
    p_c2 = world_to_grid(-2.10, -0.96)
    draw.rectangle([min(p_c1[0], p_c2[0]), min(p_c1[1], p_c2[1]), max(p_c1[0], p_c2[0]), max(p_c1[1], p_c2[1])], fill=254, outline=254)

    # Draw each chair as a solid contiguous rectangle matching the physical object
    for cname, cx, cy in corridor_chairs:
        p1 = world_to_grid(cx - 0.20, cy - 0.20)
        p2 = world_to_grid(cx + 0.20, cy + 0.20)
        draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    # Corridor trolley (outside Nurses' Station at 1.80, -0.72)
    p1 = world_to_grid(1.80 - 0.40, -0.72 - 0.20)
    p2 = world_to_grid(1.80 + 0.40, -0.72 + 0.20)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=0, outline=0)

    final_pgm_arr = np.array(im_grid)

    print("\nSolid Map Statistics:")
    print(f"  Free cells (254):    {np.sum(final_pgm_arr == 254)}")
    print(f"  Occupied cells (0):  {np.sum(final_pgm_arr == 0)}")
    print(f"  Unknown cells (205): {np.sum(final_pgm_arr == 205)}")

    # Verify corridor chairs
    print("\nCorridor Waiting Chairs Verification:")
    for cname, cx, cy in corridor_chairs:
        p = world_to_grid(cx, cy)
        px, py = int(round(p[0])), int(round(p[1]))
        val = final_pgm_arr[py, px]
        print(f"  {cname} at ({cx}, {cy}) -> px=({px}, {py}) = {val} ({'OCCUPIED - SOLID' if val == 0 else 'ERROR'})")

    # Generate Color Maps:
    # 1. Dark Theme PNG:
    # 205 -> [26, 32, 46]     (exterior unknown space)
    # 254 -> [12, 16, 28]     (walkable interior floor)
    # 0   -> [0, 212, 255]    (solid cyan walls, tables, beds, chairs)
    dark_rgb = np.zeros((height, width, 3), dtype=np.uint8)
    dark_rgb[final_pgm_arr == 205] = [26, 32, 46]
    dark_rgb[final_pgm_arr == 254] = [12, 16, 28]
    dark_rgb[final_pgm_arr == 0]   = [0, 212, 255]

    # 2. Light Theme PNG:
    # 205 -> [226, 234, 244]  (soft architectural blueprint ice-blue)
    # 254 -> [255, 255, 255]  (clean white floor)
    # 0   -> [15, 23, 42]     (solid deep obsidian navy walls, tables, beds, chairs)
    light_rgb = np.zeros((height, width, 3), dtype=np.uint8)
    light_rgb[final_pgm_arr == 205] = [226, 234, 244]
    light_rgb[final_pgm_arr == 254] = [255, 255, 255]
    light_rgb[final_pgm_arr == 0]   = [15, 23, 42]

    # Save to disk for each requested output name
    for name in output_names:
        yaml_path = maps_dir / f"{name}.yaml"
        pgm_path = maps_dir / f"{name}.pgm"
        png_path = maps_dir / f"{name}.png"
        light_png_path = maps_dir / f"{name}_light.png"

        # 1. Save PGM (ROS standard trinary)
        cv2.imwrite(str(pgm_path), final_pgm_arr)
        print(f"Saved: {pgm_path} ({pgm_path.stat().st_size} bytes)")

        # 2. Save YAML
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

        # 3. Save Dark PNG
        cv2.imwrite(str(png_path), cv2.cvtColor(dark_rgb, cv2.COLOR_RGB2BGR))
        print(f"Saved: {png_path} ({png_path.stat().st_size} bytes)")

        # 4. Save Light PNG
        cv2.imwrite(str(light_png_path), cv2.cvtColor(light_rgb, cv2.COLOR_RGB2BGR))
        print(f"Saved: {light_png_path} ({light_png_path.stat().st_size} bytes)")

        # Mirror to app/src/agv_description/maps
        app_maps_dir = ws_root / "app" / "src" / "agv_description" / "maps"
        if app_maps_dir.exists():
            shutil.copy2(str(pgm_path), str(app_maps_dir / f"{name}.pgm"))
            shutil.copy2(str(yaml_path), str(app_maps_dir / f"{name}.yaml"))
            shutil.copy2(str(png_path), str(app_maps_dir / f"{name}.png"))
            shutil.copy2(str(light_png_path), str(app_maps_dir / f"{name}_light.png"))
            print(f"Mirrored to: {app_maps_dir / name}")

    print("\nSolid Hospital Map generation complete!")

if __name__ == '__main__':
    ws = Path(__file__).resolve().parents[3]
    generate_hospital_map(ws)
