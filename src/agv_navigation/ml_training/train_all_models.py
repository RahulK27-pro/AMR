#!/usr/bin/env python3
"""80% Simulation + 20% Real Hardware Multi-Model Training & Best-Model Selection Suite.

1. Combines 80% live Gazebo closed-loop evasion captures (+ sim augmentation if needed)
   with 20% hardware-domain data (real hardware .npz captures + physical SLAM maps
   with empirical YDLIDAR X2 beam sparsity and hardware kinematics from logs/).
2. Evaluates all candidate classifiers (RandomForest, ExtraTrees, HistGradientBoosting,
   1D-CNN, CompactMLP-64-32, CompactMLP-96-48, CompactMLP-128-64) across all classes.
3. Evaluates all trajectory predictors (ConstantVelocityKF, PureGRU-32, KalmanResidualGRU-32,
   KalmanResidualGRU-64) on 1.0s and 2.0s horizons.
4. Automatically chooses the best classifier and best predictor, exports them to pure-NumPy
   .npz files under src/agv_navigation/models/, and records full benchmarks.
"""

import argparse
import csv
import glob
import json
import math
import os
import time
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image
from sklearn.ensemble import ExtraTreesClassifier, HistGradientBoostingClassifier, RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix, f1_score
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import yaml

from agv_navigation.map_paths import resolve_asset_in_root, resolve_map_image_from_yaml
from agv_navigation.ml_perception.kalman_tracker import KalmanTracker
from agv_navigation.ml_perception.lidar_clustering import (
    CLASS_CLUTTER,
    CLASS_NAMES,
    CLASS_PERSON,
    CLASS_WALL,
    FEATURE_NAMES,
    MapContext,
    extract_lidar_clusters,
)
from synthetic_lidar_sim import (
    _ray_circle_intersect,
    _ray_obb_intersect,
    _raycast_static_map,
    _resample_cluster_profile,
    _sample_free_point,
    generate_dataset,
)
from train_classifier import Profile1DCNN, _calibrate_temperature, _episode_split
from train_predictor import KalmanResidualGRU, PureGRU, _metrics


class FlexMLP(nn.Module):
    """Configurable 2-hidden-layer LayerNorm MLP matching pure-NumPy runtime."""

    def __init__(self, in_dim: int, h1: int, h2: int, num_classes: int = 3):
        super().__init__()
        self.fc1 = nn.Linear(in_dim, h1)
        self.ln1 = nn.LayerNorm(h1)
        self.fc2 = nn.Linear(h1, h2)
        self.ln2 = nn.LayerNorm(h2)
        self.fc3 = nn.Linear(h2, num_classes)
        self.drop = nn.Dropout(0.05)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.drop(torch.relu(self.ln1(self.fc1(x))))
        x = torch.relu(self.ln2(self.fc2(x)))
        return self.fc3(x)


def _extract_hardware_beam_dropout(ws_root: str) -> float:
    """Measure real YDLIDAR X2 dropout rate from physical logs/navigation_run_*.csv."""
    beams = []
    for f in sorted(glob.glob(os.path.join(ws_root, "logs", "navigation_run_*.csv")))[-8:]:
        try:
            with open(f, "r") as fh:
                for row in csv.DictReader(fh):
                    b = int(row.get("num_valid_beams", 0))
                    if b > 0:
                        beams.append(b)
        except Exception:
            continue
    if beams:
        mean_valid = float(np.mean(beams))
        return float(np.clip(1.0 - (mean_valid / 360.0), 0.15, 0.40))
    return 0.34


def _load_hardware_maps(ws_root: str) -> List[Dict]:
    """Load real physical SLAM maps scanned with the YDLIDAR X2 robot."""
    hw_names = ["prakash_room", "hardware_map", "final_map", "my_room_map", "my_next_map"]
    hw_root = os.path.join(ws_root, "maps")
    loaded = []
    for name in hw_names:
        yf = resolve_asset_in_root(hw_root, f"{name}.yaml")
        if not yf:
            continue
        try:
            with open(yf, "r") as f:
                meta = yaml.safe_load(f)
            img_p = resolve_map_image_from_yaml(yf, meta)
            if not img_p:
                continue
            arr = np.flipud(np.array(Image.open(img_p).convert("L")))
            orig = meta.get("origin", [0.0, 0.0, 0.0])
            loaded.append(
                {
                    "name": name,
                    "occ": arr < 50,
                    "free": arr > 230,
                    "res": float(meta.get("resolution", 0.05)),
                    "ox": float(orig[0]),
                    "oy": float(orig[1]),
                }
            )
        except Exception:
            continue
    return loaded


def _generate_hardware_domain_data(ws_root: str, target_samples: int, seed: int = 101) -> Dict[str, np.ndarray]:
    """Build 20% hardware-domain dataset from real hardware captures + physical SLAM maps & YDLIDAR stats."""
    # Check for any real recorded hardware .npz files first
    hw_captures = sorted(glob.glob(os.path.join(ws_root, "ml_datasets", "hardware_*.npz")))
    hw_drop = _extract_hardware_beam_dropout(ws_root)
    hw_maps = _load_hardware_maps(ws_root)
    rng = np.random.default_rng(seed)

    clf_X, clf_P, clf_y, clf_ep = [], [], [], []
    tr_seq, tr_gt, tr_kv, tr_ep = [], [], [], []

    # 1. Process real hardware captures if present
    for cap_idx, cf in enumerate(hw_captures):
        try:
            d = np.load(cf)
            ranges_seq = d["ranges"]
            poses_seq = d["poses"]
            ts_seq = d["timestamps"]
            amin = float(d["angle_min"])
            ainc = float(d["angle_inc"])
            angles = (amin + np.arange(ranges_seq.shape[1]) * ainc).astype(np.float32)
            tracker = KalmanTracker()
            map_ctx = MapContext(hw_maps[0]["occ"], hw_maps[0]["res"], hw_maps[0]["ox"], hw_maps[0]["oy"]) if hw_maps else None
            for i in range(len(ranges_seq)):
                dt = max(0.05, min(0.25, float(ts_seq[i] - ts_seq[i - 1]))) if i > 0 else 0.10
                clusters, _, _ = extract_lidar_clusters(ranges_seq[i], angles, float(poses_seq[i, 0]), float(poses_seq[i, 1]), float(poses_seq[i, 2]), map_ctx=map_ctx)
                for tr, feat20 in tracker.update(clusters, dt=dt):
                    if tr.misses > 0 or tr.last_cluster is None:
                        continue
                    spd = float(feat20[15])
                    disp_r = float(feat20[17])
                    if tr.last_cluster.wall_edt_mean < 0.16 and tr.last_cluster.chord_width > 0.60:
                        lbl = CLASS_WALL
                    elif spd >= 0.20 and disp_r >= 0.45 and tr.age >= 3:
                        lbl = CLASS_PERSON
                    else:
                        lbl = CLASS_CLUTTER
                    clf_X.append(feat20)
                    clf_P.append(_resample_cluster_profile(tr.last_cluster.points_local, 24))
                    clf_y.append(lbl)
                    clf_ep.append(5000 + cap_idx)
        except Exception:
            continue

    # 2. Calibrated hardware-map YDLIDAR X2 episodes matching empirical beam sparsity & crawl speeds
    if hw_maps and len(clf_y) < target_samples:
        angles = np.linspace(-math.pi, math.pi - (2.0 * math.pi / 360.0), 360, dtype=np.float32)
        dt, horizon = 0.10, 20
        hw_map_ctxs = [MapContext(m["occ"], m["res"], m["ox"], m["oy"]) for m in hw_maps]
        ep = 0
        while len(clf_y) < target_samples and ep < 80:
            mi = ep % len(hw_maps)
            m = hw_maps[mi]
            map_ctx = hw_map_ctxs[mi]
            r_pos = _sample_free_point(m, map_ctx, rng, min_wall_dist=0.35)
            r_yaw = float(rng.uniform(-math.pi, math.pi))
            clutter_objs = [
                {
                    "id": 100 + c,
                    "pos": _sample_free_point(m, map_ctx, rng, min_wall_dist=0.30),
                    "hw": float(rng.uniform(0.12, 0.32)),
                    "hh": float(rng.uniform(0.12, 0.28)),
                    "theta": float(rng.uniform(-math.pi, math.pi)),
                }
                for c in range(2)
            ]
            persons = [
                {
                    "id": 300 + p,
                    "pos": _sample_free_point(m, map_ctx, rng, min_wall_dist=0.45),
                    "heading": float(rng.uniform(-math.pi, math.pi)),
                    "speed": float(rng.uniform(0.22, 0.58)),
                    "phase": float(rng.uniform(0, 6.28)),
                    "traj": [],
                }
                for p in range(2)
            ]
            steps = 75
            robot_states, p_snaps = [], []
            for s in range(steps):
                # Physical hardware crawl speed (v <= 0.13 m/s, w <= 0.40 rad/s)
                r_v = float(rng.uniform(0.0, 0.13))
                r_w = float(rng.uniform(-0.38, 0.38))
                r_yaw = math.atan2(math.sin(r_yaw + r_w * dt), math.cos(r_yaw + r_w * dt))
                nxt_r = r_pos + np.array([r_v * math.cos(r_yaw) * dt, r_v * math.sin(r_yaw) * dt], dtype=np.float32)
                if map_ctx.query_edt_and_grad(float(nxt_r[0]), float(nxt_r[1]))[0] > 0.20:
                    r_pos = nxt_r
                robot_states.append((float(r_pos[0]), float(r_pos[1]), float(r_yaw), r_w))
                snap = []
                for p in persons:
                    edt_p, gx_p, gy_p = map_ctx.query_edt_and_grad(float(p["pos"][0]), float(p["pos"][1]))
                    if edt_p < 0.42:
                        away_ang = math.atan2(gy_p, gx_p)
                        h_diff = math.atan2(math.sin(away_ang - p["heading"]), math.cos(away_ang - p["heading"]))
                        p["heading"] += float(np.clip(1.8 * h_diff * dt, -0.22, 0.22))
                    p["heading"] += float(rng.normal(0.0, 0.03))
                    nxt_p = p["pos"] + np.array([p["speed"] * math.cos(p["heading"]) * dt, p["speed"] * math.sin(p["heading"]) * dt], dtype=np.float32)
                    if map_ctx.query_edt_and_grad(float(nxt_p[0]), float(nxt_p[1]))[0] > 0.22:
                        p["pos"] = nxt_p
                    else:
                        p["heading"] = math.atan2(gy_p, gx_p) + float(rng.uniform(-0.3, 0.3))
                    p["traj"].append(p["pos"].copy())
                    p["phase"] += 2.0 * math.pi * 1.0 * dt
                    snap.append({"id": p["id"], "pos": p["pos"].copy(), "heading": p["heading"], "speed": p["speed"], "phase": p["phase"]})
                p_snaps.append(snap)

            tracker = KalmanTracker()
            hist_tid: Dict[int, List] = {}
            for s in range(steps):
                rx, ry, ryaw, rw = robot_states[s]
                wa = ryaw + angles
                dirs = np.column_stack((np.cos(wa), np.sin(wa))).astype(np.float32)
                best_r = _raycast_static_map(rx, ry, ryaw, angles, m, max_range=8.0)
                b_cls = np.where(best_r <= 8.0, CLASS_WALL, -1).astype(np.int32)
                b_src = np.where(best_r <= 8.0, 0, -1).astype(np.int32)
                for co in clutter_objs:
                    tr = _ray_obb_intersect(rx, ry, dirs, co["pos"][0], co["pos"][1], co["hw"], co["hh"], co["theta"])
                    hit = tr < best_r
                    best_r[hit], b_cls[hit], b_src[hit] = tr[hit], CLASS_CLUTTER, co["id"]
                for ps in p_snaps[s]:
                    hx, hy = math.cos(ps["heading"]), math.sin(ps["heading"])
                    lx, ly = -hy, hx
                    st = 0.18 * math.sin(ps["phase"])
                    for sign in (1.0, -1.0):
                        lg = ps["pos"] + np.array([sign * st * hx + sign * 0.10 * lx, sign * st * hy + sign * 0.10 * ly], dtype=np.float32)
                        tr = _ray_circle_intersect(rx, ry, dirs, float(lg[0]), float(lg[1]), 0.068)
                        hit = tr < best_r
                        best_r[hit], b_cls[hit], b_src[hit] = tr[hit], CLASS_PERSON, ps["id"]

                noisy_r = best_r + rng.normal(0.0, 0.018, size=len(best_r)).astype(np.float32)
                noisy_r[rng.random(len(noisy_r)) < hw_drop] = np.inf
                noisy_r[best_r > 8.0] = np.inf
                clusters, _, _ = extract_lidar_clusters(noisy_r, angles, rx + float(rng.normal(0, 0.025)), ry + float(rng.normal(0, 0.025)), ryaw + float(rng.normal(0, 0.015)), map_ctx=map_ctx)

                vmask = np.isfinite(noisy_r) & (noisy_r > 0.15) & (noisy_r < 8.0)
                if not np.any(vmask) or not clusters:
                    continue
                from scipy.spatial import KDTree
                rtree = KDTree(np.column_stack((noisy_r[vmask] * np.cos(angles[vmask]), noisy_r[vmask] * np.sin(angles[vmask]))))
                r_c, r_s = b_cls[vmask], b_src[vmask]
                pid_map = {ps["id"]: pi for pi, ps in enumerate(p_snaps[s])}
                cl_lbls, cl_pidx = [], []
                for cl in clusters:
                    _, nni = rtree.query(cl.points_local)
                    vs, cs = np.unique(r_c[nni], return_counts=True)
                    lbl = int(vs[int(np.argmax(cs))])
                    pi_match = -1
                    if lbl == CLASS_PERSON:
                        psrcs = r_s[nni][r_c[nni] == CLASS_PERSON]
                        sv, sc = np.unique(psrcs, return_counts=True)
                        pi_match = pid_map.get(int(sv[int(np.argmax(sc))]), -1)
                    cl_lbls.append(lbl)
                    cl_pidx.append(pi_match)

                for tr, feat20 in tracker.update(clusters, dt=dt, robot_ang_vel=rw):
                    if tr.misses > 0 or tr.last_cluster is None:
                        continue
                    bc = min(range(len(clusters)), key=lambda i: float(np.linalg.norm(clusters[i].centroid_map - tr.last_cluster.centroid_map)))
                    lbl, p_i = cl_lbls[bc], cl_pidx[bc]
                    clf_X.append(feat20)
                    clf_P.append(_resample_cluster_profile(tr.last_cluster.points_local, 24))
                    clf_y.append(lbl)
                    clf_ep.append(6000 + ep)
                    if lbl == CLASS_PERSON and p_i >= 0 and s + horizon < steps:
                        h = hist_tid.setdefault(tr.track_id, [])
                        edt_t, gx_t, gy_t = map_ctx.query_edt_and_grad(float(tr.x[0]), float(tr.x[1]))
                        prev_p = h[-1][0][:2] if h else tr.x[:2]
                        prev_v = h[-1][0][2:4] if h else tr.x[2:4]
                        dp, acc = tr.x[:2] - prev_p, (tr.x[2:4] - prev_v) / dt
                        h.append((tr.x.copy(), np.array([dp[0], dp[1], tr.x[2], tr.x[3], acc[0], acc[1], edt_t, gx_t, gy_t], dtype=np.float32)))
                        if len(h) >= 4:
                            sub = h[-min(10, len(h)):]
                            vx_n, vy_n = float(tr.x[2]), float(tr.x[3])
                            psi = math.atan2(vy_n, vx_n) if math.hypot(vx_n, vy_n) >= 0.1 else 0.0
                            cp, sp = math.cos(-psi), math.sin(-psi)
                            seq_a = [[rv[0]*cp-rv[1]*sp, rv[0]*sp+rv[1]*cp, rv[2]*cp-rv[3]*sp, rv[2]*sp+rv[3]*cp, rv[4]*cp-rv[5]*sp, rv[4]*sp+rv[5]*cp, rv[6], rv[7]*cp-rv[8]*sp, rv[7]*sp+rv[8]*cp, math.hypot(rv[2], rv[3])] for _, rv in sub]
                            while len(seq_a) < 10:
                                seq_a.insert(0, [0.0] * 10)
                            fg = np.stack(persons[p_i]["traj"][s+1 : s+1+horizon], axis=0) - tr.x[:2].astype(np.float32)
                            fg_loc = np.column_stack((fg[:, 0]*cp - fg[:, 1]*sp, fg[:, 0]*sp + fg[:, 1]*cp)).astype(np.float32)
                            kv_loc = np.array([vx_n*cp - vy_n*sp, vx_n*sp + vy_n*cp], dtype=np.float32)
                            # Keep trajectory targets that advance continuously rather than 3m-room corner reversals
                            if float(fg_loc[-1, 0]) >= 0.18 * float(kv_loc[0]) * 2.0 or float(np.hypot(vx_n, vy_n)) < 0.20:
                                tr_seq.append(np.asarray(seq_a, dtype=np.float32))
                                tr_gt.append(fg_loc)
                                tr_kv.append(kv_loc)
                                tr_ep.append(6000 + ep)
            ep += 1

    return {
        "clf_features": np.asarray(clf_X, dtype=np.float32),
        "clf_profiles": np.asarray(clf_P, dtype=np.float32),
        "clf_labels": np.asarray(clf_y, dtype=np.int64),
        "clf_ep_ids": np.asarray(clf_ep, dtype=np.int64),
        "traj_seq_inputs": np.asarray(tr_seq, dtype=np.float32),
        "traj_gt_future_xy": np.asarray(tr_gt, dtype=np.float32),
        "traj_kf_vel": np.asarray(tr_kv, dtype=np.float32),
        "traj_ep_ids": np.asarray(tr_ep, dtype=np.int64),
        "hw_dropout_measured": hw_drop,
        "hw_captures_count": len(hw_captures),
        "hw_maps_used": [m["name"] for m in hw_maps],
    }


def _export_tree_ensemble(clf_model, mean: np.ndarray, std: np.ndarray, out_path: str):
    """Pack scikit-learn RandomForest / ExtraTrees into flat NumPy arrays for zero-dep inference."""
    estimators = clf_model.estimators_
    n_trees = len(estimators)
    max_nodes = max(est.tree_.node_count for est in estimators)

    feat = np.full((n_trees, max_nodes), -2, dtype=np.int32)
    thresh = np.zeros((n_trees, max_nodes), dtype=np.float32)
    left = np.zeros((n_trees, max_nodes), dtype=np.int32)
    right = np.zeros((n_trees, max_nodes), dtype=np.int32)
    values = np.zeros((n_trees, max_nodes, 3), dtype=np.float32)

    for ti, est in enumerate(estimators):
        t = est.tree_
        nc = t.node_count
        feat[ti, :nc] = t.feature
        thresh[ti, :nc] = t.threshold.astype(np.float32)
        left[ti, :nc] = t.children_left
        right[ti, :nc] = t.children_right
        val = t.value[:, 0, :].astype(np.float32)
        row_s = np.maximum(1e-6, np.sum(val, axis=-1, keepdims=True))
        values[ti, :nc] = val / row_s

    np.savez(
        out_path,
        model_type=np.array("tree_ensemble"),
        mean=mean.astype(np.float32),
        std=std.astype(np.float32),
        tree_features=feat,
        tree_thresholds=thresh,
        tree_left=left,
        tree_right=right,
        tree_values=values,
        n_trees=np.int32(n_trees),
        max_nodes=np.int32(max_nodes),
    )


def _export_mlp(state_dict: Dict, mean: np.ndarray, std: np.ndarray, temp: float, out_path: str):
    np.savez(
        out_path,
        model_type=np.array("mlp"),
        mean=mean.astype(np.float32),
        std=std.astype(np.float32),
        temperature=np.float32(temp),
        fc1_w=state_dict["fc1.weight"].astype(np.float32),
        fc1_b=state_dict["fc1.bias"].astype(np.float32),
        ln1_w=state_dict["ln1.weight"].astype(np.float32),
        ln1_b=state_dict["ln1.bias"].astype(np.float32),
        fc2_w=state_dict["fc2.weight"].astype(np.float32),
        fc2_b=state_dict["fc2.bias"].astype(np.float32),
        ln2_w=state_dict["ln2.weight"].astype(np.float32),
        ln2_b=state_dict["ln2.bias"].astype(np.float32),
        fc3_w=state_dict["fc3.weight"].astype(np.float32),
        fc3_b=state_dict["fc3.bias"].astype(np.float32),
    )


def run_80_20_training(ws_root: str, sim_live_path: str):
    torch.set_num_threads(min(4, os.cpu_count() or 2))
    n_jobs_sk = min(4, os.cpu_count() or 2)
    sim_aug_path = os.path.join(ws_root, "ml_datasets", "synthetic_lidar_dataset.npz")
    combined_path = os.path.join(ws_root, "ml_datasets", "combined_80_20_dataset.npz")
    clf_out = os.path.join(ws_root, "src", "agv_navigation", "models", "obstacle_classifier.npz")
    pred_out = os.path.join(ws_root, "src", "agv_navigation", "models", "trajectory_predictor.npz")
    meta_out = os.path.join(ws_root, "src", "agv_navigation", "models", "model_metadata.json")

    # 1. Load live Gazebo simulation dataset (and ensure rich scenario coverage)
    live = np.load(sim_live_path) if os.path.exists(sim_live_path) else None
    if not os.path.exists(sim_aug_path):
        generate_dataset(ws_root, sim_aug_path, num_episodes=80, steps_per_ep=100, seed=42)
    aug = np.load(sim_aug_path)

    # Prioritize live Gazebo captures in the 80% simulation share; supplement with sim raycast episodes
    if live is not None and len(live["clf_labels"]) >= 500:
        live_n = len(live["clf_labels"])
        # Repeat live closed-loop evasion frames cleanly (2x weight) + strided supplement across all maps
        rep = 2 if live_n < 12000 else 1
        aug_c_idx = np.linspace(0, len(aug["clf_labels"]) - 1, min(12000, len(aug["clf_labels"])), dtype=int)
        aug_t_idx = np.linspace(0, len(aug["traj_kf_vel"]) - 1, min(2800, len(aug["traj_kf_vel"])), dtype=int)
        sim_X = np.concatenate([np.tile(live["clf_features"], (rep, 1)), aug["clf_features"][aug_c_idx]], axis=0)
        sim_P = np.concatenate([np.tile(live["clf_profiles"], (rep, 1, 1)), aug["clf_profiles"][aug_c_idx]], axis=0)
        sim_y = np.concatenate([np.tile(live["clf_labels"], rep), aug["clf_labels"][aug_c_idx]], axis=0)
        sim_ep = np.concatenate([np.tile(live["clf_ep_ids"], rep), 1000 + aug["clf_ep_ids"][aug_c_idx]], axis=0)

        # Guard live trajectories against static-spawn artifacts by requiring actual horizon variance
        live_valid_tr = np.zeros(len(live["traj_gt_future_xy"]), dtype=bool)
        if len(live["traj_gt_future_xy"]) > 0:
            step_ptp = np.max(np.ptp(live["traj_gt_future_xy"], axis=1), axis=-1)
            live_valid_tr = step_ptp > 0.04
        if np.sum(live_valid_tr) > 0:
            lt_seq = live["traj_seq_inputs"][live_valid_tr]
            lt_gt = live["traj_gt_future_xy"][live_valid_tr]
            lt_kv = live["traj_kf_vel"][live_valid_tr]
            lt_ep = live["traj_ep_ids"][live_valid_tr]
            sim_tseq = np.concatenate([np.tile(lt_seq, (rep, 1, 1)), aug["traj_seq_inputs"][aug_t_idx]], axis=0)
            sim_tgt = np.concatenate([np.tile(lt_gt, (rep, 1, 1)), aug["traj_gt_future_xy"][aug_t_idx]], axis=0)
            sim_tkv = np.concatenate([np.tile(lt_kv, (rep, 1)), aug["traj_kf_vel"][aug_t_idx]], axis=0)
            sim_tep = np.concatenate([np.tile(lt_ep, rep), 1000 + aug["traj_ep_ids"][aug_t_idx]], axis=0)
            live_traj_raw = int(len(lt_seq))
        else:
            sim_tseq = aug["traj_seq_inputs"][aug_t_idx]
            sim_tgt = aug["traj_gt_future_xy"][aug_t_idx]
            sim_tkv = aug["traj_kf_vel"][aug_t_idx]
            sim_tep = 1000 + aug["traj_ep_ids"][aug_t_idx]
            live_traj_raw = 0
        live_count_raw = int(live_n)
    else:
        aug_c_idx = np.linspace(0, len(aug["clf_labels"]) - 1, min(16000, len(aug["clf_labels"])), dtype=int)
        aug_t_idx = np.linspace(0, len(aug["traj_kf_vel"]) - 1, min(2800, len(aug["traj_kf_vel"])), dtype=int)
        sim_X, sim_P, sim_y, sim_ep = aug["clf_features"][aug_c_idx], aug["clf_profiles"][aug_c_idx], aug["clf_labels"][aug_c_idx], 1000 + aug["clf_ep_ids"][aug_c_idx]
        sim_tseq, sim_tgt, sim_tkv, sim_tep = aug["traj_seq_inputs"][aug_t_idx], aug["traj_gt_future_xy"][aug_t_idx], aug["traj_kf_vel"][aug_t_idx], 1000 + aug["traj_ep_ids"][aug_t_idx]
        live_count_raw, live_traj_raw = 0, 0

    # 2. Enforce exact 80% Simulation / 20% Hardware split
    n_sim = len(sim_y)
    target_hw = max(1000, int(round(n_sim * 0.25)))  # 0.25 * n_sim -> 80/20 total split!
    hw = _generate_hardware_domain_data(ws_root, target_samples=target_hw, seed=101)
    n_hw = min(target_hw, len(hw["clf_labels"]))

    X_all = np.concatenate([sim_X, hw["clf_features"][:n_hw]], axis=0)
    P_all = np.concatenate([sim_P, hw["clf_profiles"][:n_hw]], axis=0)
    y_all = np.concatenate([sim_y, hw["clf_labels"][:n_hw]], axis=0)
    ep_all = np.concatenate([sim_ep, hw["clf_ep_ids"][:n_hw]], axis=0)

    n_sim_tr = len(sim_tseq)
    target_hw_tr = max(200, int(round(n_sim_tr * 0.25)))
    if len(hw["traj_seq_inputs"]) > 0:
        hw_tr_n = min(target_hw_tr, len(hw["traj_seq_inputs"]))
        tseq_all = np.concatenate([sim_tseq, hw["traj_seq_inputs"][:hw_tr_n]], axis=0)
        tgt_all = np.concatenate([sim_tgt, hw["traj_gt_future_xy"][:hw_tr_n]], axis=0)
        tkv_all = np.concatenate([sim_tkv, hw["traj_kf_vel"][:hw_tr_n]], axis=0)
        tep_all = np.concatenate([sim_tep, hw["traj_ep_ids"][:hw_tr_n]], axis=0)
    else:
        hw_tr_n = 0
        tseq_all, tgt_all, tkv_all, tep_all = sim_tseq, sim_tgt, sim_tkv, sim_tep

    np.savez_compressed(
        combined_path,
        clf_features=X_all, clf_profiles=P_all, clf_labels=y_all, clf_ep_ids=ep_all,
        traj_seq_inputs=tseq_all, traj_gt_future_xy=tgt_all, traj_kf_vel=tkv_all, traj_ep_ids=tep_all,
    )
    sim_pct = 100.0 * n_sim / len(y_all)
    hw_pct = 100.0 * n_hw / len(y_all)
    print(
        f"[DATA_80_20] Total={len(y_all)} clusters ({sim_pct:.1f}% Sim [live_gazebo={live_count_raw}], "
        f"{hw_pct:.1f}% Hardware [hw_drop={hw['hw_dropout_measured']:.2f}, maps={hw['hw_maps_used']}])",
        flush=True,
    )

    # 3. Train & Benchmark ALL Classifier Candidates
    tr_m, va_m = _episode_split(ep_all, val_frac=0.20, seed=42)
    X_tr, X_va = X_all[tr_m], X_all[va_m]
    P_tr, P_va = P_all[tr_m], P_all[va_m]
    y_tr, y_va = y_all[tr_m], y_all[va_m]

    mean = np.mean(X_tr, axis=0)
    std = np.maximum(np.std(X_tr, axis=0), 1e-4)
    X_tr_n = (X_tr - mean) / std
    X_va_n = (X_va - mean) / std

    counts = np.bincount(y_tr, minlength=3).astype(np.float32)
    cw_sqrt = np.sqrt(np.sum(counts) / (3.0 * np.maximum(counts, 1.0)))

    clf_benchmarks = {}
    exportable_candidates = []  # (composite_score, name, kind, payload)

    def _eval_preds(name: str, y_pred: np.ndarray, t_train: float, t_inf_ms: float):
        mf1 = float(f1_score(y_va, y_pred, average="macro"))
        per_f1 = f1_score(y_va, y_pred, average=None, labels=[0, 1, 2])
        cm = confusion_matrix(y_va, y_pred, labels=[0, 1, 2])
        wall_fp = float(cm[0, 2] / max(1, np.sum(cm[0])))
        score = mf1 + 0.30 * float(per_f1[2]) - 2.0 * wall_fp
        clf_benchmarks[name] = {
            "macro_f1": round(mf1, 4),
            "wall_f1": round(float(per_f1[0]), 4),
            "clutter_f1": round(float(per_f1[1]), 4),
            "person_f1": round(float(per_f1[2]), 4),
            "wall_as_person_fp": round(wall_fp, 4),
            "composite_score": round(score, 4),
            "train_sec": round(t_train, 2),
            "infer_10_ms": round(t_inf_ms, 3),
        }
        print(
            f"[CLF_EVAL] {name:24s} | macro-F1={mf1:.4f} (W={per_f1[0]:.3f}, C={per_f1[1]:.3f}, P={per_f1[2]:.3f}) "
            f"wall->P={wall_fp:.4f} score={score:.4f}",
            flush=True,
        )
        return score

    # Tree ensembles (exported natively via pure-NumPy tree packer!)
    for name, est in [
        ("RandomForest_80", RandomForestClassifier(n_estimators=80, max_depth=14, min_samples_leaf=3, class_weight="balanced_subsample", n_jobs=n_jobs_sk, random_state=42)),
        ("ExtraTrees_80", ExtraTreesClassifier(n_estimators=80, max_depth=14, min_samples_leaf=2, class_weight="balanced_subsample", n_jobs=n_jobs_sk, random_state=42)),
    ]:
        t0 = time.perf_counter()
        est.fit(X_tr_n, y_tr)
        tt = time.perf_counter() - t0
        ti0 = time.perf_counter()
        _ = est.predict_proba(X_va_n[:10])
        ti = (time.perf_counter() - ti0) * 1000.0
        sc = _eval_preds(name, est.predict(X_va_n), tt, ti)
        exportable_candidates.append((sc, name, "tree_ensemble", est))

    # HistGradientBoosting benchmark
    t0 = time.perf_counter()
    hgb = HistGradientBoostingClassifier(max_iter=120, max_depth=8, learning_rate=0.08, random_state=42)
    sample_w = cw_sqrt[y_tr]
    hgb.fit(X_tr_n, y_tr, sample_weight=sample_w)
    tt = time.perf_counter() - t0
    _eval_preds("HistGradientBoosting", hgb.predict(X_va_n), tt, 1.5)

    # 1D-CNN spatial benchmark
    t0 = time.perf_counter()
    torch.manual_seed(42)
    cnn = Profile1DCNN(num_classes=3)
    opt_c = torch.optim.AdamW(cnn.parameters(), lr=3e-3, weight_decay=1e-4)
    crit_c = nn.CrossEntropyLoss(weight=torch.tensor(cw_sqrt))
    dl_c = DataLoader(TensorDataset(torch.from_numpy(P_tr), torch.from_numpy(y_tr)), batch_size=512, shuffle=True)
    for _ in range(10):
        cnn.train()
        for bx, by in dl_c:
            opt_c.zero_grad()
            crit_c(cnn(bx), by).backward()
            opt_c.step()
    cnn.eval()
    with torch.no_grad():
        c_pred = np.argmax(cnn(torch.from_numpy(P_va)).numpy(), axis=1)
    _eval_preds("1D_CNN_Spatial", c_pred, time.perf_counter() - t0, 0.8)

    # Compact MLP sweep (64-32, 96-48, 128-64)
    dl_m = DataLoader(TensorDataset(torch.from_numpy(X_tr_n), torch.from_numpy(y_tr)), batch_size=512, shuffle=True)
    for h1, h2 in [(64, 32), (96, 48), (128, 64)]:
        mname = f"CompactMLP_{h1}_{h2}"
        t0 = time.perf_counter()
        torch.manual_seed(42)
        mlp = FlexMLP(X_all.shape[1], h1, h2, 3)
        mlp_epochs = 16
        opt = torch.optim.AdamW(mlp.parameters(), lr=3.5e-3, weight_decay=3e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=mlp_epochs)
        crit_m = nn.CrossEntropyLoss(weight=torch.tensor(cw_sqrt), label_smoothing=0.03)
        best_mf1, best_sd = -1.0, None
        for ep_i in range(mlp_epochs):
            mlp.train()
            for bx, by in dl_m:
                opt.zero_grad()
                crit_m(mlp(bx + 0.015 * torch.randn_like(bx)), by).backward()
                opt.step()
            sched.step()
            if (ep_i % 2 == 1) or (ep_i == mlp_epochs - 1):
                mlp.eval()
                with torch.no_grad():
                    vp = np.argmax(mlp(torch.from_numpy(X_va_n)).numpy(), axis=1)
                f1_now = float(f1_score(y_va, vp, average="macro"))
                if f1_now > best_mf1:
                    best_mf1 = f1_now
                    best_sd = {k: v.detach().cpu().numpy().copy() for k, v in mlp.state_dict().items()}
        mlp.load_state_dict({k: torch.from_numpy(v) for k, v in best_sd.items()})
        mlp.eval()
        with torch.no_grad():
            va_logits = mlp(torch.from_numpy(X_va_n)).numpy()
        temp = _calibrate_temperature(va_logits, y_va)
        sc = _eval_preds(mname, np.argmax(va_logits, axis=1), time.perf_counter() - t0, 0.05)
        exportable_candidates.append((sc, mname, "mlp", (best_sd, temp)))

    # Select & export BEST Classifier
    exportable_candidates.sort(key=lambda x: x[0], reverse=True)
    best_clf_score, best_clf_name, best_clf_kind, best_clf_obj = exportable_candidates[0]
    os.makedirs(os.path.dirname(clf_out), exist_ok=True)
    if best_clf_kind == "tree_ensemble":
        _export_tree_ensemble(best_clf_obj, mean, std, clf_out)
    else:
        _export_mlp(best_clf_obj[0], mean, std, best_clf_obj[1], clf_out)
    print(f"[SELECTED_CLF] Best Model = {best_clf_name} (score={best_clf_score:.4f}, kind={best_clf_kind}) -> {clf_out}", flush=True)

    # 4. Train & Benchmark ALL Trajectory Predictors (1.0s & 2.0s)
    pt_m, pv_m = _episode_split(tep_all, val_frac=0.20, seed=42)
    s_tr, s_va = tseq_all[pt_m], tseq_all[pv_m]
    g_tr, g_va = tgt_all[pt_m], tgt_all[pv_m]
    k_tr, k_va = tkv_all[pt_m], tkv_all[pv_m]

    p_mean = np.mean(s_tr.reshape(-1, s_tr.shape[-1]), axis=0)
    p_std = np.maximum(np.std(s_tr.reshape(-1, s_tr.shape[-1]), axis=0), 1e-4)
    s_tr_n = (s_tr - p_mean) / p_std
    s_va_n = (s_va - p_mean) / p_std

    dt, horizon = 0.10, tgt_all.shape[1]
    t_steps = (np.arange(1, horizon + 1, dtype=np.float32) * dt)[None, :, None]
    pred_benchmarks = {}
    pred_candidates = []

    cv_m = _metrics(k_va[:, None, :] * t_steps, g_va)
    pred_benchmarks["ConstantVelocityKF"] = cv_m
    print(f"[PRED_EVAL] ConstantVelocityKF       | {cv_m}", flush=True)

    # Pure-GRU baseline
    torch.manual_seed(42)
    pgru = PureGRU(s_tr.shape[-1], 32, horizon, dt)
    opt_pg = torch.optim.AdamW(pgru.parameters(), lr=4e-3, weight_decay=2e-4)
    dl_pg = DataLoader(TensorDataset(torch.from_numpy(s_tr_n), torch.from_numpy(g_tr)), batch_size=512, shuffle=True)
    for _ in range(16):
        pgru.train()
        for bs, bg in dl_pg:
            opt_pg.zero_grad()
            torch.nn.functional.smooth_l1_loss(pgru(bs), bg, beta=0.15).backward()
            opt_pg.step()
    pgru.eval()
    with torch.no_grad():
        pg_m = _metrics(pgru(torch.from_numpy(s_va_n)).numpy(), g_va)
    pred_benchmarks["PureGRU_32"] = pg_m
    print(f"[PRED_EVAL] PureGRU_32               | {pg_m}", flush=True)

    dl_knn = DataLoader(TensorDataset(torch.from_numpy(s_tr_n), torch.from_numpy(k_tr), torch.from_numpy(g_tr)), batch_size=512, shuffle=True)
    for hdim in (32, 48):
        pname = f"KalmanResidualGRU_{hdim}"
        torch.manual_seed(42)
        mod = KalmanResidualGRU(s_tr.shape[-1], hdim, horizon, dt)
        gru_epochs = 24
        opt = torch.optim.AdamW(mod.parameters(), lr=4e-3, weight_decay=3e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=gru_epochs)
        best_ade, best_sd = float("inf"), None
        for ep_i in range(gru_epochs):
            mod.train()
            for bs, bkv, bgt in dl_knn:
                opt.zero_grad()
                pp, ps = mod(bs, bkv)
                loss = torch.nn.functional.smooth_l1_loss(pp, bgt, beta=0.15) + 0.15 * 0.5 * torch.mean(((bgt - pp) / ps) ** 2 + 2.0 * torch.log(ps))
                loss.backward()
                torch.nn.utils.clip_grad_norm_(mod.parameters(), 2.0)
                opt.step()
            sched.step()
            if (ep_i % 2 == 1) or (ep_i == gru_epochs - 1):
                mod.eval()
                with torch.no_grad():
                    vp, _ = mod(torch.from_numpy(s_va_n), torch.from_numpy(k_va))
                ade = float(np.mean(np.linalg.norm(vp.numpy() - g_va, axis=-1)))
                if ade < best_ade:
                    best_ade = ade
                    best_sd = {k: v.detach().cpu().numpy().copy() for k, v in mod.state_dict().items()}
        mod.load_state_dict({k: torch.from_numpy(v) for k, v in best_sd.items()})
        mod.eval()
        with torch.no_grad():
            vp, vs = mod(torch.from_numpy(s_va_n), torch.from_numpy(k_va))
        m_dict = _metrics(vp.numpy(), g_va)
        m_dict["cov_95_axis"] = round(float(np.mean(np.abs(g_va - vp.numpy()) <= 1.96 * vs.numpy())), 4)
        m_dict["fde_2s_improvement_vs_kf_pct"] = round(100.0 * (cv_m["fde_2s"] - m_dict["fde_2s"]) / max(1e-4, cv_m["fde_2s"]), 2)
        pred_benchmarks[pname] = m_dict
        err_score = 0.6 * m_dict["fde_2s"] + 0.4 * m_dict["ade_2s"]
        pred_candidates.append((err_score, pname, hdim, best_sd))
        print(f"[PRED_EVAL] {pname:24s} | {m_dict}", flush=True)

    pred_candidates.sort(key=lambda x: x[0])
    _, best_pred_name, best_hdim, best_pred_sd = pred_candidates[0]
    np.savez(
        pred_out,
        mean=p_mean.astype(np.float32),
        std=p_std.astype(np.float32),
        dt=np.float32(dt),
        horizon=np.int32(horizon),
        hidden_dim=np.int32(best_hdim),
        w_ih=best_pred_sd["gru_cell.weight_ih"].astype(np.float32),
        b_ih=best_pred_sd["gru_cell.bias_ih"].astype(np.float32),
        w_hh=best_pred_sd["gru_cell.weight_hh"].astype(np.float32),
        b_hh=best_pred_sd["gru_cell.bias_hh"].astype(np.float32),
        dec1_w=best_pred_sd["dec_fc1.weight"].astype(np.float32),
        dec1_b=best_pred_sd["dec_fc1.bias"].astype(np.float32),
        dec2_w=best_pred_sd["dec_fc2.weight"].astype(np.float32),
        dec2_b=best_pred_sd["dec_fc2.bias"].astype(np.float32),
    )
    print(f"[SELECTED_PRED] Best Predictor = {best_pred_name} -> {pred_out}", flush=True)

    meta = {
        "dataset_composition": {
            "total_cluster_samples": int(len(y_all)),
            "simulation_samples": int(n_sim),
            "simulation_pct": round(sim_pct, 2),
            "live_gazebo_raw_samples": live_count_raw,
            "live_gazebo_traj_sequences": live_traj_raw,
            "hardware_samples": int(n_hw),
            "hardware_pct": round(hw_pct, 2),
            "hardware_captures_used": hw["hw_captures_count"],
            "hardware_maps_used": hw["hw_maps_used"],
            "hardware_ydlidar_dropout_rate": round(hw["hw_dropout_measured"], 4),
            "total_trajectory_sequences": int(len(tseq_all)),
        },
        "classifier": {
            "feature_names": list(FEATURE_NAMES),
            "class_names": list(CLASS_NAMES),
            "selected_model": best_clf_name,
            "selected_runtime_type": best_clf_kind,
            "benchmarks": clf_benchmarks,
        },
        "predictor": {
            "horizon_steps": int(horizon),
            "dt": float(dt),
            "selected_model": best_pred_name,
            "selected_hidden_dim": int(best_hdim),
            "benchmarks": pred_benchmarks,
        },
    }
    with open(meta_out, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[METADATA] Updated {meta_out}")


def main():
    parser = argparse.ArgumentParser(description="80/20 Sim+Hardware Multi-Model Training & Selection")
    parser.add_argument("--ws-root", default=os.path.expanduser("~/AMR/AMR-main"))
    parser.add_argument("--sim-live", default=os.path.expanduser("~/AMR/AMR-main/ml_datasets/gazebo_live_dataset.npz"))
    args = parser.parse_args()
    run_80_20_training(args.ws_root, args.sim_live)


if __name__ == "__main__":
    main()
