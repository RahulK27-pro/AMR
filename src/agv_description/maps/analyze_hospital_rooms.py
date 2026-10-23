#!/usr/bin/env python3
import sys
import cv2
import numpy as np
import yaml
from pathlib import Path

maps_dir = Path(__file__).resolve().parent
with open(maps_dir / "hospital_map.yaml") as f:
    info = yaml.safe_load(f)

img = cv2.imread(str(maps_dir / "hospital_map.pgm"), cv2.IMREAD_GRAYSCALE)
h, w = img.shape
res = float(info['resolution'])
ox, oy = info['origin'][:2]

_, binary = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)
dt = cv2.distanceTransform(binary, cv2.DIST_L2, 5)

# Room boundary boxes: [xmin, xmax, ymin, ymax]
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

print(f"{'Room Name':26s} | {'Total Px':8s} | {'Free Px':8s} | {'Occ Px':7s} | {'Max Clear':9s} | {'Best Free Spot (X, Y)'}")
print("-" * 88)

for name, (x0, x1, y0, y1) in room_bounds.items():
    gx0 = int(round((x0 - ox) / res))
    gx1 = int(round((x1 - ox) / res))
    gy0 = int(round((y0 - oy) / res))
    gy1 = int(round((y1 - oy) / res))
    py0 = (h - 1) - gy1
    py1 = (h - 1) - gy0
    
    sub = binary[py0:py1+1, gx0:gx1+1]
    sub_dt = dt[py0:py1+1, gx0:gx1+1]
    
    total = sub.size
    free = int(np.sum(sub == 255))
    occ = total - free
    max_c = float(np.max(sub_dt) * res)
    
    # Find position of max clearance
    best_y, best_x = np.unravel_index(np.argmax(sub_dt), sub_dt.shape)
    best_px = gx0 + best_x
    best_py = py0 + best_y
    best_wx = ox + best_px * res
    best_wy = oy + (h - 1 - best_py) * res
    
    print(f"{name:26s} | {total:8d} | {free:8d} | {occ:7d} | {max_c:7.2f} m | ({best_wx:+5.2f}, {best_wy:+5.2f})")
