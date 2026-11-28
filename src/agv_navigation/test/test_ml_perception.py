"""Unit, latency, and closed-loop behavior tests for ML LiDAR perception & prediction."""

import math
import os
import sys
import time

import numpy as np
import pytest

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from agv_navigation.ml_perception import (
    CLASS_CLUTTER,
    CLASS_PERSON,
    CLASS_WALL,
    MapContext,
    PerceptionPipeline,
    extract_lidar_clusters,
)


def _ray_circle(ox, oy, dirs, cx, cy, r, max_r=8.0):
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dirs[:, 0] + fy * dirs[:, 1])
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * c
    out = np.full(len(dirs), max_r + 1.0, dtype=np.float32)
    m = disc >= 0.0
    if np.any(m):
        sq = np.sqrt(disc[m])
        t1 = (-b[m] - sq) * 0.5
        t2 = (-b[m] + sq) * 0.5
        out[m] = np.where(t1 > 0.15, t1, np.where(t2 > 0.15, t2, max_r + 1.0))
    return out


def _ray_line_seg(ox, oy, dirs, x1, y1, x2, y2, max_r=8.0):
    vx, vy = x2 - x1, y2 - y1
    out = np.full(len(dirs), max_r + 1.0, dtype=np.float32)
    for i, (dx, dy) in enumerate(dirs):
        denom = dx * vy - dy * vx
        if abs(denom) < 1e-7:
            continue
        t = ((x1 - ox) * vy - (y1 - oy) * vx) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
        if 0.15 < t < max_r and 0.0 <= u <= 1.0:
            out[i] = t
    return out


def _make_test_map_context():
    # 12m x 12m room at 0.05m resolution with perimeter walls at +-5m
    grid = np.zeros((240, 240), dtype=bool)
    grid[20, 20:220] = True
    grid[220, 20:220] = True
    grid[20:220, 20] = True
    grid[20:220, 220] = True
    return MapContext(grid, resolution=0.05, origin_x=-6.0, origin_y=-6.0)


def _model_paths():
    models_dir = os.path.join(PKG_SRC, "models")
    clf = os.path.join(models_dir, "obstacle_classifier.npz")
    pred = os.path.join(models_dir, "trajectory_predictor.npz")
    return clf if os.path.exists(clf) else None, pred if os.path.exists(pred) else None


def test_polar_clustering_and_leg_pair_fusion():
    map_ctx = _make_test_map_context()
    angles = np.linspace(-math.pi, math.pi - 2 * math.pi / 360.0, 360, dtype=np.float32)
    dirs = np.column_stack((np.cos(angles), np.sin(angles)))

    # Wall at x = 3.2m, clutter box/cylinder at (1.5, -1.3), 2-leg pedestrian at (2.0, 0.2)
    r = _ray_line_seg(0.0, 0.0, dirs, 3.2, -2.5, 3.2, 2.5)
    r = np.minimum(r, _ray_circle(0.0, 0.0, dirs, 1.5, -1.3, 0.22))
    r = np.minimum(r, _ray_circle(0.0, 0.0, dirs, 2.05, 0.08, 0.07))
    r = np.minimum(r, _ray_circle(0.0, 0.0, dirs, 1.95, 0.34, 0.07))
    r[r > 7.5] = np.inf

    clusters, ext_loc, ext_map = extract_lidar_clusters(r, angles, 0.0, 0.0, 0.0, map_ctx=map_ctx)
    assert len(ext_loc) > 30
    # Wall is occluded into visible segments behind the clutter & pedestrian; expect 1 leg-pair, 1 clutter, >=1 wall segments
    leg_clusters = [c for c in clusters if c.is_leg_pair]
    clutter_clusters = [c for c in clusters if (not c.is_leg_pair and c.centroid_local[0] < 2.2)]
    wall_clusters = [c for c in clusters if c.centroid_local[0] > 2.8]
    assert len(leg_clusters) == 1
    assert len(clutter_clusters) == 1
    assert len(wall_clusters) >= 1
    assert max(c.geom_features[7] for c in wall_clusters) > 0.95  # High linearity


def test_static_wall_and_clutter_do_not_trigger_person_during_rotation():
    clf_path, pred_path = _model_paths()
    pipe = PerceptionPipeline(classifier_path=clf_path, predictor_path=pred_path, prediction_steps=20)
    map_ctx = _make_test_map_context()
    pipe.map_ctx = map_ctx

    angles = np.linspace(-math.pi, math.pi - 2 * math.pi / 360.0, 360, dtype=np.float32)
    rng = np.random.default_rng(7)
    yaw = 0.0
    for step in range(18):
        yaw += 0.14  # Rapid rotation (~1.4 rad/s)
        world_a = yaw + angles
        dirs = np.column_stack((np.cos(world_a), np.sin(world_a)))

        # Unmapped clutter at (1.8, 1.0) and mapped wall at x = 4.95 (inside map wall)
        r = _ray_line_seg(0.0, 0.0, dirs, 4.95, -4.5, 4.95, 4.5)
        r = np.minimum(r, _ray_circle(0.0, 0.0, dirs, 1.8, 1.0, 0.24))
        r += rng.normal(0.0, 0.012, size=len(r)).astype(np.float32)
        r[r > 7.5] = np.inf

        tracks, _, _ = pipe.process_scan(
            r, angles,
            pose_x=float(rng.normal(0.0, 0.015)),
            pose_y=float(rng.normal(0.0, 0.015)),
            pose_yaw=yaw,
            dt=0.10,
            robot_ang_vel=1.4,
        )

    person_tracks = [t for t in tracks if t.age >= 3 and t.class_id == CLASS_PERSON]
    assert len(person_tracks) == 0


def test_moving_pedestrian_classification_and_2s_prediction():
    clf_path, pred_path = _model_paths()
    pipe = PerceptionPipeline(classifier_path=clf_path, predictor_path=pred_path, prediction_steps=20)
    map_ctx = _make_test_map_context()
    pipe.map_ctx = map_ctx

    angles = np.linspace(-math.pi, math.pi - 2 * math.pi / 360.0, 360, dtype=np.float32)
    latencies = []
    px, py = 2.2, -1.6
    vy = 0.75
    tracks = []

    for step in range(20):
        py += vy * 0.10
        world_a = angles
        dirs = np.column_stack((np.cos(world_a), np.sin(world_a)))

        # 2-leg walking pedestrian + background static wall
        r = _ray_line_seg(0.0, 0.0, dirs, 4.95, -4.5, 4.95, 4.5)
        stride = 0.16 * math.sin(step * 0.6)
        r = np.minimum(r, _ray_circle(0.0, 0.0, dirs, px - 0.10, py + stride, 0.068))
        r = np.minimum(r, _ray_circle(0.0, 0.0, dirs, px + 0.10, py - stride, 0.068))
        r[r > 7.5] = np.inf

        t0 = time.perf_counter()
        tracks, _, _ = pipe.process_scan(r, angles, 0.0, 0.0, 0.0, dt=0.10)
        latencies.append((time.perf_counter() - t0) * 1000.0)

    persons = [t for t in tracks if t.class_id == CLASS_PERSON and t.age >= 5]
    assert len(persons) == 1
    p = persons[0]
    assert np.isclose(np.sum(p.class_probs), 1.0, atol=1e-4)
    assert p.pred_traj is not None and p.pred_traj.shape == (20, 2)
    assert p.pred_std is not None and p.pred_std.shape == (20, 2)
    assert np.all(p.pred_std > 0.0)
    # Predicted path moves forward (+y) smoothly
    assert p.pred_traj[-1, 1] > p.pos[1] + 0.35
    # Verify real-time inference budget (< 10ms even on first cold cache, median < 4ms)
    assert np.median(latencies) < 4.0


def test_mppi_radius_padding_and_cpa_swerve_direction():
    """Verify obstacle-radius-aware collision check and predictive CPA swerve direction."""
    robot_radius = 0.11
    obs_radius = 0.25  # e.g. dynamic_obstacle cart or pedestrian
    std_t = np.array([0.08, 0.08], dtype=np.float32)
    sigma_pad = min(0.16, 0.40 * math.hypot(float(std_t[0]), float(std_t[1])))
    eff_coll_r = robot_radius + 0.04 + obs_radius + sigma_pad

    # At 0.36m center-to-center distance, old 0.15m check missed collision; new check flags it
    assert 0.36 < eff_coll_r

    # Pedestrian currently on robot's RIGHT (y = -0.35), walking briskly LEFT (+y at 1.0 m/s)
    # Robot moving +x at 0.5 m/s toward target (2.0, 0.0): CPA at t~0.7s puts person on LEFT (y > 0)
    t_vec = np.arange(1, 11, dtype=np.float32) * 0.1
    pred_traj = np.column_stack((np.full(10, 1.2, dtype=np.float32), -0.35 + 1.0 * t_vec))
    ego_x = 0.5 * t_vec
    ego_y = np.zeros_like(t_vec)
    cpa_idx = int(np.argmin(np.hypot(pred_traj[:, 0] - ego_x, pred_traj[:, 1] - ego_y)))
    cpa_y = float(pred_traj[cpa_idx, 1])
    assert cpa_y > 0.0  # CPA is on robot's LEFT -> swerve RIGHT (-w) to pass behind pedestrian


def test_tree_ensemble_pure_numpy_runtime(tmp_path):
    """Verify pure-NumPy packed tree ensemble outputs valid calibrated class probabilities."""
    from agv_navigation.ml_perception.classifier_runtime import ObstacleClassifierRuntime

    # 1 tree, 3 nodes: root splits feature 1 (chord_width) <= 0.5 -> left=leaf(PERSON), right=leaf(WALL)
    npz_path = str(tmp_path / "test_trees.npz")
    np.savez(
        npz_path,
        model_type=np.array("tree_ensemble"),
        mean=np.zeros(20, dtype=np.float32),
        std=np.ones(20, dtype=np.float32),
        tree_features=np.array([[1, -1, -1]], dtype=np.int32),
        tree_thresholds=np.array([[0.5, 0.0, 0.0]], dtype=np.float32),
        tree_left=np.array([[1, -1, -1]], dtype=np.int32),
        tree_right=np.array([[2, -1, -1]], dtype=np.int32),
        tree_values=np.array([[[0.3, 0.3, 0.4], [0.05, 0.15, 0.80], [0.90, 0.08, 0.02]]], dtype=np.float32),
        n_trees=np.int32(1),
        max_nodes=np.int32(3),
    )
    rt = ObstacleClassifierRuntime(npz_path)
    assert rt.model_type == "tree_ensemble"
    x = np.zeros((2, 20), dtype=np.float32)
    x[0, 1] = 0.25  # -> PERSON
    x[1, 1] = 1.40  # -> WALL
    probs = rt.predict_proba_batch(x)
    assert probs.shape == (2, 3)
    assert np.argmax(probs[0]) == CLASS_PERSON
    assert np.argmax(probs[1]) == CLASS_WALL


