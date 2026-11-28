"""Offline synthetic 2D LiDAR dataset generator using real workspace occupancy maps.

Generates realistic 360-beam scans with YDLIDAR/Gazebo noise, AMCL pose jitter,
sliding wall-aperture segments, unmapped walls, static clutter, and bipedal/cart
dynamic obstacles for training both the obstacle classifier and GRU trajectory predictor.
"""

import argparse
import glob
import math
import os
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image
from scipy.spatial import KDTree
import yaml

from agv_navigation.ml_perception.kalman_tracker import KalmanTracker
from agv_navigation.ml_perception.lidar_clustering import (
    CLASS_CLUTTER,
    CLASS_PERSON,
    CLASS_WALL,
    MapContext,
    extract_lidar_clusters,
)


def load_workspace_maps(ws_root: str) -> List[Dict]:
    """Load occupancy maps (occ boolean grid, resolution, origin) from workspace."""
    yaml_files = sorted(
        glob.glob(os.path.join(ws_root, "src", "agv_description", "maps", "*.yaml"))
        + glob.glob(os.path.join(ws_root, "maps", "*.yaml"))
    )
    loaded = []
    seen_names = set()
    for yf in yaml_files:
        stem = os.path.splitext(os.path.basename(yf))[0]
        if stem in seen_names or "filtered" in stem:
            continue
        try:
            with open(yf, "r") as f:
                meta = yaml.safe_load(f)
            img_path = meta.get("image", "")
            if not os.path.isabs(img_path):
                img_path = os.path.join(os.path.dirname(yf), img_path)
            if not os.path.exists(img_path):
                continue
            arr = np.flipud(np.array(Image.open(img_path).convert("L")))
            occ = arr < 50
            free = arr > 230
            if np.sum(free) < 100 or np.sum(occ) < 40:
                continue
            orig = meta.get("origin", [0.0, 0.0, 0.0])
            loaded.append(
                {
                    "name": stem,
                    "occ": occ,
                    "free": free,
                    "res": float(meta.get("resolution", 0.05)),
                    "ox": float(orig[0]),
                    "oy": float(orig[1]),
                }
            )
            seen_names.add(stem)
        except Exception:
            continue
    return loaded


def _sample_free_point(map_dict: Dict, map_ctx: MapContext, rng: np.random.Generator, min_wall_dist: float = 0.55) -> np.ndarray:
    """Sample a random (x, y) in free space at least min_wall_dist from walls."""
    ys, xs = np.where(map_ctx.edt_grid >= min_wall_dist)
    if len(xs) == 0:
        ys, xs = np.where(map_dict["free"])
    idx = int(rng.integers(0, len(xs)))
    px = map_dict["ox"] + (xs[idx] + 0.5) * map_dict["res"]
    py = map_dict["oy"] + (ys[idx] + 0.5) * map_dict["res"]
    return np.array([px, py], dtype=np.float32)


def _raycast_static_map(
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    angles_local: np.ndarray,
    map_dict: Dict,
    max_range: float = 8.0,
    step_m: float = 0.03,
) -> np.ndarray:
    """Vectorized 360-beam raycast against binary occupancy grid."""
    r_steps = np.arange(0.16, max_range, step_m, dtype=np.float32)
    world_angles = (pose_yaw + angles_local)[:, None]
    rx = pose_x + np.cos(world_angles) * r_steps[None, :]
    ry = pose_y + np.sin(world_angles) * r_steps[None, :]

    gx = np.floor((rx - map_dict["ox"]) / map_dict["res"]).astype(np.int32)
    gy = np.floor((ry - map_dict["oy"]) / map_dict["res"]).astype(np.int32)
    h, w = map_dict["occ"].shape
    inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
    hits = np.zeros_like(inside, dtype=bool)
    hits[inside] = map_dict["occ"][gy[inside], gx[inside]]
    # Treat outside-of-map as open/max_range unless hit
    any_hit = np.any(hits, axis=1)
    first_idx = np.argmax(hits, axis=1)
    ranges = np.full(len(angles_local), max_range + 1.0, dtype=np.float32)
    ranges[any_hit] = r_steps[first_idx[any_hit]]
    return ranges


def _ray_circle_intersect(
    ox: float,
    oy: float,
    dirs: np.ndarray,
    cx: float,
    cy: float,
    radius: float,
    max_range: float = 8.0,
) -> np.ndarray:
    """Vectorized ray-circle intersection returning nearest positive t per ray."""
    fx = ox - cx
    fy = oy - cy
    b = 2.0 * (fx * dirs[:, 0] + fy * dirs[:, 1])
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * c
    out = np.full(len(dirs), max_range + 1.0, dtype=np.float32)
    mask = disc >= 0.0
    if np.any(mask):
        sq = np.sqrt(disc[mask])
        t1 = (-b[mask] - sq) * 0.5
        t2 = (-b[mask] + sq) * 0.5
        valid = np.where(t1 > 0.15, t1, np.where(t2 > 0.15, t2, max_range + 1.0))
        out[mask] = np.minimum(out[mask], valid)
    return out


def _ray_obb_intersect(
    ox: float,
    oy: float,
    dirs: np.ndarray,
    cx: float,
    cy: float,
    half_w: float,
    half_h: float,
    theta: float,
    max_range: float = 8.0,
) -> np.ndarray:
    """Vectorized ray-oriented-bounding-box intersection."""
    ct = math.cos(-theta)
    st = math.sin(-theta)
    dx = ox - cx
    dy = oy - cy
    ro_x = dx * ct - dy * st
    ro_y = dx * st + dy * ct
    rd_x = dirs[:, 0] * ct - dirs[:, 1] * st
    rd_y = dirs[:, 0] * st + dirs[:, 1] * ct

    eps = 1e-7
    inv_x = 1.0 / np.where(np.abs(rd_x) > eps, rd_x, np.copysign(eps, rd_x))
    inv_y = 1.0 / np.where(np.abs(rd_y) > eps, rd_y, np.copysign(eps, rd_y))

    tx1 = (-half_w - ro_x) * inv_x
    tx2 = (half_w - ro_x) * inv_x
    ty1 = (-half_h - ro_y) * inv_y
    ty2 = (half_h - ro_y) * inv_y

    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = (tmax >= np.maximum(tmin, 0.15)) & (tmin < max_range)
    out = np.full(len(dirs), max_range + 1.0, dtype=np.float32)
    out[hit] = np.where(tmin[hit] > 0.15, tmin[hit], tmax[hit])
    return out


def _resample_cluster_profile(pts_local: np.ndarray, num_bins: int = 24) -> np.ndarray:
    """Resample ordered local cluster points into a fixed (num_bins, 3) tensor for 1D-CNN ablation."""
    n = len(pts_local)
    centroid = np.mean(pts_local, axis=0)
    centered = pts_local - centroid
    # Rotate so cluster chord aligns along X
    chord = pts_local[-1] - pts_local[0]
    ang = math.atan2(float(chord[1]), float(chord[0]))
    ca, sa = math.cos(-ang), math.sin(-ang)
    rot_x = centered[:, 0] * ca - centered[:, 1] * sa
    rot_y = centered[:, 0] * sa + centered[:, 1] * ca
    r_vals = np.linalg.norm(pts_local, axis=-1)

    t_src = np.linspace(0.0, 1.0, n)
    t_dst = np.linspace(0.0, 1.0, num_bins)
    rx = np.interp(t_dst, t_src, rot_x)
    ry = np.interp(t_dst, t_src, rot_y)
    rr = np.interp(t_dst, t_src, r_vals)
    return np.stack((rx, ry, rr), axis=-1).astype(np.float32)


def generate_dataset(
    ws_root: str,
    out_path: str,
    num_episodes: int = 120,
    steps_per_ep: int = 110,
    seed: int = 42,
):
    """Simulate diverse episodes and save classification + trajectory prediction datasets."""
    rng = np.random.default_rng(seed)
    maps = load_workspace_maps(ws_root)
    if not maps:
        raise RuntimeError(f"No valid maps found under {ws_root}")

    angles = np.linspace(-math.pi, math.pi - (2.0 * math.pi / 360.0), 360, dtype=np.float32)
    dt = 0.10
    horizon = 20

    clf_features: List[np.ndarray] = []
    clf_profiles: List[np.ndarray] = []
    clf_labels: List[int] = []
    clf_ep_ids: List[int] = []

    traj_seq_inputs: List[np.ndarray] = []     # (seq_len, 10)
    traj_seq_headings: List[float] = []
    traj_gt_future_xy: List[np.ndarray] = []   # (horizon, 2) world delta from last pos
    traj_kf_vel: List[np.ndarray] = []         # (2,)
    traj_ep_ids: List[int] = []

    large_maps = [m for m in maps if min(m["occ"].shape) * m["res"] >= 5.0]
    small_maps = [m for m in maps if min(m["occ"].shape) * m["res"] < 5.0]
    for ep in range(num_episodes):
        if large_maps and small_maps:
            # 65% wide corridor/warehouse maps for long uninterrupted human walks, 35% tight physical rooms
            if (ep % 20) < 13:
                m = large_maps[ep % len(large_maps)]
            else:
                m = small_maps[ep % len(small_maps)]
        else:
            m = maps[ep % len(maps)]
        map_ctx = MapContext(m["occ"], m["res"], m["ox"], m["oy"])
        is_large_map = (min(m["occ"].shape) * m["res"]) >= 5.0

        # Robot initial pose & motion pattern (mix of moving and stationary)
        r_pos = _sample_free_point(m, map_ctx, rng, min_wall_dist=0.45)
        r_yaw = float(rng.uniform(-math.pi, math.pi))
        robot_static = (ep % 5 == 0)

        # Spawn unmapped clutter boxes/cylinders
        clutter_objs = []
        n_clutter = int(rng.integers(2, 6))
        for cid in range(1, n_clutter + 1):
            cpos = _sample_free_point(m, map_ctx, rng, min_wall_dist=0.35)
            kind = "box" if rng.random() < 0.65 else "cyl"
            clutter_objs.append(
                {
                    "id": 100 + cid,
                    "kind": kind,
                    "pos": cpos,
                    "hw": float(rng.uniform(0.11, 0.42)),
                    "hh": float(rng.uniform(0.10, 0.35)),
                    "theta": float(rng.uniform(-math.pi, math.pi)),
                    "r": float(rng.uniform(0.14, 0.30)),
                }
            )

        # Spawn 1-2 unmapped partition walls (distinctly wall-scale: 2.2m to 4.8m long)
        unmapped_walls = []
        for wid in range(1, int(rng.integers(1, 3)) + 1):
            wpos = _sample_free_point(m, map_ctx, rng, min_wall_dist=0.60)
            unmapped_walls.append(
                {
                    "id": 200 + wid,
                    "pos": wpos,
                    "hw": float(rng.uniform(1.10, 2.40)),
                    "hh": 0.08,
                    "theta": float(rng.uniform(-math.pi, math.pi)),
                }
            )

        # Spawn dynamic PERSON agents (bipedal walkers + moving carts)
        persons = []
        n_persons = int(rng.integers(2, 5))
        patterns = ["cruise", "cruise", "curved", "stop_go", "turn_back"]
        min_dim_m = min(m["occ"].shape) * m["res"]
        max_p_spd = 0.72 if min_dim_m < 5.0 else 1.25
        for pid in range(1, n_persons + 1):
            ppos = _sample_free_point(m, map_ctx, rng, min_wall_dist=0.58)
            heading = float(rng.uniform(-math.pi, math.pi))
            speed = float(rng.uniform(0.32, max_p_spd))
            pat = patterns[int(rng.integers(0, len(patterns)))]
            is_biped = bool(rng.random() < 0.78)
            persons.append(
                {
                    "id": 300 + pid,
                    "pos": ppos.copy(),
                    "heading": heading,
                    "speed": speed,
                    "target_speed": speed,
                    "pattern": pat,
                    "omega": float(rng.uniform(-0.55, 0.55)) if pat == "curved" else 0.0,
                    "phase": float(rng.uniform(0.0, 2.0 * math.pi)),
                    "gait_freq": float(rng.uniform(0.85, 1.35)),
                    "is_biped": is_biped,
                    "cart_r": float(rng.uniform(0.20, 0.28)),
                    "pause_timer": 0.0,
                    "traj": [],
                }
            )

        # Simulate episode trajectory first so we have exact future ground truth
        robot_states = []
        person_snapshots = []  # per step list of dicts
        for step in range(steps_per_ep):
            # Move robot safely away from walls
            if not robot_static:
                edt_r, gx_r, gy_r = map_ctx.query_edt_and_grad(float(r_pos[0]), float(r_pos[1]))
                if edt_r < 0.42:
                    away_yaw = math.atan2(gy_r, gx_r)
                    diff = math.atan2(math.sin(away_yaw - r_yaw), math.cos(away_yaw - r_yaw))
                    r_w = float(np.clip(2.0 * diff, -1.4, 1.4))
                    r_v = 0.12
                else:
                    r_w = float(0.45 * math.sin(0.07 * step + ep))
                    r_v = float(rng.uniform(0.18, 0.52))
                r_yaw = math.atan2(math.sin(r_yaw + r_w * dt), math.cos(r_yaw + r_w * dt))
                cand_p = r_pos + np.array([r_v * math.cos(r_yaw) * dt, r_v * math.sin(r_yaw) * dt], dtype=np.float32)
                if map_ctx.query_edt_and_grad(float(cand_p[0]), float(cand_p[1]))[0] > 0.22:
                    r_pos = cand_p
                else:
                    r_yaw += 1.2
            else:
                r_w = 0.0
            robot_states.append((float(r_pos[0]), float(r_pos[1]), float(r_yaw), float(r_w)))

            snap = []
            for p in persons:
                edt_p, gx_p, gy_p = map_ctx.query_edt_and_grad(float(p["pos"][0]), float(p["pos"][1]))
                # Smooth social wall-avoidance steering
                if edt_p < 0.45:
                    away_ang = math.atan2(gy_p, gx_p)
                    h_diff = math.atan2(math.sin(away_ang - p["heading"]), math.cos(away_ang - p["heading"]))
                    p["heading"] += float(np.clip(1.8 * h_diff * dt, -0.22, 0.22))

                if p["pattern"] == "turn_back":
                    if p["pause_timer"] > 0.0:
                        p["speed"] = max(0.0, p["speed"] - 1.5 * dt)
                        p["pause_timer"] -= dt
                        if p["pause_timer"] <= 0.0:
                            p["heading"] += math.pi + float(rng.normal(0.0, 0.14))
                    else:
                        p["speed"] = min(p["target_speed"], p["speed"] + 1.2 * dt)
                        if step > 10 and (step % 55 == 0):
                            p["pause_timer"] = float(rng.uniform(0.6, 1.2))
                elif p["pattern"] == "curved":
                    p["heading"] += p["omega"] * dt
                elif p["pattern"] == "stop_go":
                    cycle = step % 55
                    if 24 <= cycle < 30:
                        p["speed"] = max(0.12, p["speed"] - 1.2 * dt)
                    else:
                        p["speed"] = min(p["target_speed"], p["speed"] + 1.3 * dt)

                p["heading"] = math.atan2(math.sin(p["heading"]), math.cos(p["heading"]))
                vx = p["speed"] * math.cos(p["heading"])
                vy = p["speed"] * math.sin(p["heading"])
                nxt = p["pos"] + np.array([vx * dt, vy * dt], dtype=np.float32)
                if map_ctx.query_edt_and_grad(float(nxt[0]), float(nxt[1]))[0] > 0.24:
                    p["pos"] = nxt
                else:
                    away_ang = math.atan2(gy_p, gx_p)
                    p["heading"] = away_ang + float(rng.uniform(-0.35, 0.35))
                p["traj"].append(p["pos"].copy())
                p["phase"] += 2.0 * math.pi * p["gait_freq"] * dt
                snap.append(
                    {
                        "id": p["id"],
                        "pos": p["pos"].copy(),
                        "heading": p["heading"],
                        "speed": p["speed"],
                        "phase": p["phase"],
                        "is_biped": p["is_biped"],
                        "cart_r": p["cart_r"],
                    }
                )
            person_snapshots.append(snap)

        # Replay sensor observations with realistic AMCL drift & LiDAR noise
        tracker = KalmanTracker()
        # Track predictor history per ground-truth person id mapped from matched tracker ids
        gt_history_by_tid: Dict[int, List[Tuple[np.ndarray, float, int, int]]] = {}

        for step in range(steps_per_ep):
            rx, ry, ryaw, rw = robot_states[step]
            world_angles = ryaw + angles
            dirs = np.column_stack((np.cos(world_angles), np.sin(world_angles))).astype(np.float32)

            best_r = _raycast_static_map(rx, ry, ryaw, angles, m, max_range=8.0)
            beam_src = np.where(best_r <= 8.0, 0, -1).astype(np.int32)
            beam_class = np.where(best_r <= 8.0, CLASS_WALL, -1).astype(np.int32)

            for uw in unmapped_walls:
                tr = _ray_obb_intersect(rx, ry, dirs, uw["pos"][0], uw["pos"][1], uw["hw"], uw["hh"], uw["theta"])
                hit = tr < best_r
                best_r[hit] = tr[hit]
                beam_src[hit] = uw["id"]
                beam_class[hit] = CLASS_WALL

            for co in clutter_objs:
                if co["kind"] == "box":
                    tr = _ray_obb_intersect(rx, ry, dirs, co["pos"][0], co["pos"][1], co["hw"], co["hh"], co["theta"])
                else:
                    tr = _ray_circle_intersect(rx, ry, dirs, co["pos"][0], co["pos"][1], co["r"])
                hit = tr < best_r
                best_r[hit] = tr[hit]
                beam_src[hit] = co["id"]
                beam_class[hit] = CLASS_CLUTTER

            for ps in person_snapshots[step]:
                if ps["is_biped"]:
                    hx, hy = math.cos(ps["heading"]), math.sin(ps["heading"])
                    lx, ly = -hy, hx
                    stride = 0.14 * math.sin(ps["phase"]) if ps["speed"] > 0.1 else 0.03
                    leg1 = ps["pos"] + np.array([stride * hx + 0.09 * lx, stride * hy + 0.09 * ly], dtype=np.float32)
                    leg2 = ps["pos"] + np.array([-stride * hx - 0.09 * lx, -stride * hy - 0.09 * ly], dtype=np.float32)
                    for leg in (leg1, leg2):
                        tr = _ray_circle_intersect(rx, ry, dirs, float(leg[0]), float(leg[1]), 0.068)
                        hit = tr < best_r
                        best_r[hit] = tr[hit]
                        beam_src[hit] = ps["id"]
                        beam_class[hit] = CLASS_PERSON
                else:
                    tr = _ray_circle_intersect(rx, ry, dirs, float(ps["pos"][0]), float(ps["pos"][1]), ps["cart_r"])
                    hit = tr < best_r
                    best_r[hit] = tr[hit]
                    beam_src[hit] = ps["id"]
                    beam_class[hit] = CLASS_PERSON

            # Add realistic YDLIDAR X2 noise & 2.5% dropout
            noisy_r = best_r + rng.normal(0.0, 0.014, size=len(best_r)).astype(np.float32)
            drop_mask = rng.random(len(noisy_r)) < 0.025
            noisy_r[drop_mask] = np.inf
            noisy_r[best_r > 8.0] = np.inf

            # Simulate realistic AMCL pose estimation error
            est_x = rx + float(rng.normal(0.0, 0.025))
            est_y = ry + float(rng.normal(0.0, 0.025))
            est_yaw = ryaw + float(rng.normal(0.0, 0.015))

            clusters, _, _ = extract_lidar_clusters(
                noisy_r,
                angles,
                est_x,
                est_y,
                est_yaw,
                range_min=0.15,
                range_max=8.0,
                map_ctx=map_ctx,
            )

            # Match clusters to exact ray-level GT class & source ID (zero proximity label noise)
            valid_ray_mask = np.isfinite(noisy_r) & (noisy_r > 0.15) & (noisy_r < 8.0)
            ray_pts_loc = np.column_stack(
                (noisy_r[valid_ray_mask] * np.cos(angles[valid_ray_mask]), noisy_r[valid_ray_mask] * np.sin(angles[valid_ray_mask]))
            )
            ray_cls = beam_class[valid_ray_mask]
            ray_src = beam_src[valid_ray_mask]
            ray_tree = KDTree(ray_pts_loc) if len(ray_pts_loc) > 0 else None
            pid_to_idx = {ps["id"]: pi for pi, ps in enumerate(person_snapshots[step])}

            cluster_gt_labels = []
            cluster_gt_person_idx = []
            for cl in clusters:
                if ray_tree is None:
                    cluster_gt_labels.append(CLASS_WALL)
                    cluster_gt_person_idx.append(-1)
                    continue
                _, nn_idx = ray_tree.query(cl.points_local)
                pt_cls = ray_cls[nn_idx]
                pt_src = ray_src[nn_idx]
                vals, cnts = np.unique(pt_cls, return_counts=True)
                best_lbl = int(vals[int(np.argmax(cnts))])
                best_p_idx = -1
                if best_lbl == CLASS_PERSON:
                    p_srcs = pt_src[pt_cls == CLASS_PERSON]
                    s_vals, s_cnts = np.unique(p_srcs, return_counts=True)
                    best_sid = int(s_vals[int(np.argmax(s_cnts))])
                    best_p_idx = pid_to_idx.get(best_sid, -1)
                cluster_gt_labels.append(best_lbl)
                cluster_gt_person_idx.append(best_p_idx)

            active_tracks = tracker.update(clusters, dt=dt, robot_ang_vel=rw)
            for tr, feat20 in active_tracks:
                if tr.misses > 0 or tr.last_cluster is None:
                    continue
                # Find which cluster matched this track (closest centroid_map)
                best_c = min(
                    range(len(clusters)),
                    key=lambda idx: float(np.linalg.norm(clusters[idx].centroid_map - tr.last_cluster.centroid_map)),
                )
                lbl = cluster_gt_labels[best_c]
                p_idx = cluster_gt_person_idx[best_c]

                # Save classification sample after initial frame (include age=1..N for robust early prediction)
                clf_features.append(feat20)
                clf_profiles.append(_resample_cluster_profile(tr.last_cluster.points_local, 24))
                clf_labels.append(lbl)
                clf_ep_ids.append(ep)

                # Collect trajectory prediction sequence sample for PERSON tracks with >= 20 future GT frames
                # Skip tiny cluttered 3m room bounces when trajectory immediately reverses backward in place
                if lbl == CLASS_PERSON and p_idx >= 0 and step + horizon < steps_per_ep and (is_large_map or tr.last_cluster.wall_edt_mean >= 0.42):
                    hist = gt_history_by_tid.setdefault(tr.track_id, [])
                    edt_t, gx_t, gy_t = map_ctx.query_edt_and_grad(float(tr.x[0]), float(tr.x[1]))
                    prev_p = hist[-1][0][:2] if hist else tr.x[:2]
                    prev_v = hist[-1][0][2:4] if hist else tr.x[2:4]
                    dp = tr.x[:2] - prev_p
                    acc = (tr.x[2:4] - prev_v) / dt
                    raw_vec = np.array(
                        [dp[0], dp[1], tr.x[2], tr.x[3], acc[0], acc[1], edt_t, gx_t, gy_t],
                        dtype=np.float32,
                    )
                    hist.append((tr.x.copy(), raw_vec, p_idx, step))
                    if len(hist) >= 4:
                        seq_len = min(10, len(hist))
                        sub = hist[-seq_len:]
                        # Align sequence to current KF velocity heading (or trajectory displacement)
                        vx_now, vy_now = float(tr.x[2]), float(tr.x[3])
                        spd_now = math.hypot(vx_now, vy_now)
                        if spd_now >= 0.10:
                            psi = math.atan2(vy_now, vx_now)
                        else:
                            disp_vec = tr.x[:2] - sub[0][0][:2]
                            psi = math.atan2(float(disp_vec[1]), float(disp_vec[0])) if np.linalg.norm(disp_vec) > 0.05 else 0.0
                        cp, sp = math.cos(-psi), math.sin(-psi)
                        seq_aligned = []
                        for _, rv, _, _ in sub:
                            dx_l = rv[0] * cp - rv[1] * sp
                            dy_l = rv[0] * sp + rv[1] * cp
                            vx_l = rv[2] * cp - rv[3] * sp
                            vy_l = rv[2] * sp + rv[3] * cp
                            ax_l = rv[4] * cp - rv[5] * sp
                            ay_l = rv[4] * sp + rv[5] * cp
                            gx_l = rv[7] * cp - rv[8] * sp
                            gy_l = rv[7] * sp + rv[8] * cp
                            spd_l = math.hypot(vx_l, vy_l)
                            seq_aligned.append([dx_l, dy_l, vx_l, vy_l, ax_l, ay_l, rv[6], gx_l, gy_l, spd_l])

                        # Pad front to fixed length 10 if needed
                        while len(seq_aligned) < 10:
                            seq_aligned.insert(0, [0.0] * 10)
                        fut_gt = np.stack(persons[p_idx]["traj"][step + 1 : step + 1 + horizon], axis=0) - tr.x[:2].astype(np.float32)
                        fut_local = np.column_stack(
                            (
                                fut_gt[:, 0] * cp - fut_gt[:, 1] * sp,
                                fut_gt[:, 0] * sp + fut_gt[:, 1] * cp,
                            )
                        ).astype(np.float32)
                        kf_v_local = np.array([vx_now * cp - vy_now * sp, vx_now * sp + vy_now * cp], dtype=np.float32)

                        traj_seq_inputs.append(np.asarray(seq_aligned, dtype=np.float32))
                        traj_seq_headings.append(psi)
                        traj_gt_future_xy.append(fut_local)
                        traj_kf_vel.append(kf_v_local)
                        traj_ep_ids.append(ep)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez_compressed(
        out_path,
        clf_features=np.asarray(clf_features, dtype=np.float32),
        clf_profiles=np.asarray(clf_profiles, dtype=np.float32),
        clf_labels=np.asarray(clf_labels, dtype=np.int64),
        clf_ep_ids=np.asarray(clf_ep_ids, dtype=np.int64),
        traj_seq_inputs=np.asarray(traj_seq_inputs, dtype=np.float32),
        traj_gt_future_xy=np.asarray(traj_gt_future_xy, dtype=np.float32),
        traj_kf_vel=np.asarray(traj_kf_vel, dtype=np.float32),
        traj_ep_ids=np.asarray(traj_ep_ids, dtype=np.int64),
    )
    print(
        f"[SIM_DATASET] Saved {len(clf_labels)} cluster samples "
        f"(WALL={int(np.sum(np.asarray(clf_labels)==0))}, "
        f"CLUTTER={int(np.sum(np.asarray(clf_labels)==1))}, "
        f"PERSON={int(np.sum(np.asarray(clf_labels)==2))}) "
        f"and {len(traj_seq_inputs)} trajectory sequences -> {out_path}"
    )


def main():
    parser = argparse.ArgumentParser(description="Generate synthetic 2D LiDAR dataset for ML perception")
    parser.add_argument("--ws-root", default=os.path.expanduser("~/AMR/AMR-main"))
    parser.add_argument("--out", default=os.path.expanduser("~/AMR/AMR-main/ml_datasets/synthetic_lidar_dataset.npz"))
    parser.add_argument("--episodes", type=int, default=120)
    parser.add_argument("--steps", type=int, default=110)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    generate_dataset(args.ws_root, args.out, args.episodes, args.steps, args.seed)


if __name__ == "__main__":
    main()
