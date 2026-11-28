#!/usr/bin/env python3
"""Agent 3: Live Data Collector, 80/20 Sim+Hardware Mixer & Best-Model Trainer Agent.

Subscribes to live Gazebo /scan + /tf + /ml_pipeline/active_scenario ground truth while
the robot evades obstacles in RViz/Gazebo. Once collection finishes, builds an 80% live
simulation + 20% real-hardware dataset, trains & benchmarks all candidate models, selects
the best classifier & predictor, exports pure-NumPy weights, and hot-reloads route_runner.
"""

import json
import math
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from agv_navigation.map_paths import resolve_map_image_from_yaml, resolve_map_yaml
from agv_navigation.ml_perception.kalman_tracker import KalmanTracker
from agv_navigation.ml_perception.lidar_clustering import (
    CLASS_CLUTTER,
    CLASS_PERSON,
    CLASS_WALL,
    MapContext,
    extract_lidar_clusters,
)


def _resample_cluster_profile(pts_local: np.ndarray, num_bins: int = 24) -> np.ndarray:
    n = len(pts_local)
    centroid = np.mean(pts_local, axis=0)
    centered = pts_local - centroid
    chord = pts_local[-1] - pts_local[0]
    ang = math.atan2(float(chord[1]), float(chord[0]))
    ca, sa = math.cos(-ang), math.sin(-ang)
    rot_x = centered[:, 0] * ca - centered[:, 1] * sa
    rot_y = centered[:, 0] * sa + centered[:, 1] * ca
    r_vals = np.linalg.norm(pts_local, axis=-1)
    t_src = np.linspace(0.0, 1.0, n)
    t_dst = np.linspace(0.0, 1.0, num_bins)
    return np.stack((np.interp(t_dst, t_src, rot_x), np.interp(t_dst, t_src, rot_y), np.interp(t_dst, t_src, r_vals)), axis=-1).astype(np.float32)


class DatasetTrainerAgent(Node):
    """Collects live closed-loop Gazebo evasion samples and orchestrates 80/20 model selection."""

    def __init__(self):
        super().__init__("dataset_trainer_agent")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("ws_root", os.path.expanduser("~/AMR/AMR-main"))
        self.declare_parameter("map_name", "hospital_map")
        self.declare_parameter("collect_duration_sec", 110.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("min_sim_frames", 250)
        self.declare_parameter("auto_train", True)

        self.ws_root = str(self.get_parameter("ws_root").value)
        self.map_name = str(self.get_parameter("map_name").value)
        self.collect_duration_sec = float(self.get_parameter("collect_duration_sec").value)
        self.min_sim_frames = int(self.get_parameter("min_sim_frames").value)
        self.auto_train = bool(self.get_parameter("auto_train").value)

        self.sim_dataset_path = os.path.join(self.ws_root, "ml_datasets", "gazebo_live_dataset.npz")
        os.makedirs(os.path.dirname(self.sim_dataset_path), exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_ctx = MapContext()
        self._load_initial_map()
        self.tracker = KalmanTracker()

        self.active_scenario: Dict = {"scenario_idx": 0, "dynamic_obstacles": [], "clutter_obstacles": []}
        self.mission_done = False
        self.last_scan_sec: Optional[float] = None
        self.start_wall = time.monotonic()
        self.last_log_wall = 0.0
        self.training_started = False
        self.training_finished = False

        # Live captured frame buffers for delayed future trajectory extraction
        self.frames: List[Dict] = []
        self.person_gt_by_scenario: Dict[int, List[Tuple[float, float, float]]] = {}  # scenario_idx -> [(t, x, y)]

        latched_map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched_map_qos)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/ml_pipeline/active_scenario", self._scenario_cb, 10)
        self.create_subscription(String, "/ml_pipeline/mission_status", self._mission_cb, 10)

        self.status_pub = self.create_publisher(String, "/ml_pipeline/collector_status", 10)
        self.reload_pub = self.create_publisher(String, "/ml_pipeline/reload_models", 10)

        self.timer = self.create_timer(0.5, self._monitor_tick)
        self.get_logger().info(
            f"[AGENT_3: COLLECTOR_TRAINER] Collecting live Gazebo closed-loop evasion data "
            f"(target={self.collect_duration_sec:.0f}s, min_frames={self.min_sim_frames})."
        )

    def _load_initial_map(self):
        yp = resolve_map_yaml(self.map_name, fallbacks=('hospital_map', 'warehouse_map'), ws_root=self.ws_root)
        if not yp:
            return
        try:
            with open(yp, "r") as f:
                meta = yaml.safe_load(f)
            img_p = resolve_map_image_from_yaml(yp, meta)
            if not img_p:
                return
            arr = np.flipud(np.array(Image.open(img_p).convert("L")))
            orig = meta.get("origin", [0.0, 0.0, 0.0])
            self.map_ctx.update(arr < 50, float(meta.get("resolution", 0.05)), float(orig[0]), float(orig[1]))
        except Exception:
            pass

    def _map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.map_ctx.update(grid > 50, float(msg.info.resolution), float(msg.info.origin.position.x), float(msg.info.origin.position.y))

    def _scenario_cb(self, msg: String):
        try:
            self.active_scenario = json.loads(msg.data)
        except Exception:
            pass

    def _mission_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            if d.get("done"):
                self.mission_done = True
        except Exception:
            pass

    @staticmethod
    def _quat_to_yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_cb(self, msg: LaserScan):
        if self.training_started:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id if msg.header.frame_id else "base_link",
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.04),
            )
            px = float(t.transform.translation.x)
            py = float(t.transform.translation.y)
            pyaw = self._quat_to_yaw(t.transform.rotation)
        except TransformException:
            return

        now_sec = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        dt = max(0.05, min(0.30, now_sec - self.last_scan_sec)) if self.last_scan_sec is not None else 0.10
        self.last_scan_sec = now_sec

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = (msg.angle_min + np.arange(len(ranges)) * msg.angle_increment).astype(np.float32)

        clusters, _, _ = extract_lidar_clusters(
            ranges=ranges,
            angles=angles,
            pose_x=px,
            pose_y=py,
            pose_yaw=pyaw,
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            map_ctx=self.map_ctx if self.map_ctx.edt_grid is not None else None,
        )
        if not clusters:
            return

        sc_idx = int(self.active_scenario.get("scenario_idx", 0))
        dyn_gt = self.active_scenario.get("dynamic_obstacles", [])
        clutter_gt = self.active_scenario.get("clutter_obstacles", [])

        for dg in dyn_gt:
            self.person_gt_by_scenario.setdefault(sc_idx, []).append((now_sec, float(dg["x"]), float(dg["y"])))

        active_tracks = self.tracker.update(clusters, dt=dt, robot_ang_vel=0.0)
        frame_samples = []
        for tr, feat20 in active_tracks:
            if tr.misses > 0 or tr.last_cluster is None:
                continue
            cx, cy = float(tr.x[0]), float(tr.x[1])
            lbl = CLASS_WALL
            matched_dyn = False

            for dg in dyn_gt:
                if math.hypot(cx - float(dg["x"]), cy - float(dg["y"])) < 0.72:
                    lbl = CLASS_PERSON
                    matched_dyn = True
                    break

            if not matched_dyn:
                for cg in clutter_gt:
                    if math.hypot(cx - float(cg["x"]), cy - float(cg["y"])) < 0.65:
                        lbl = CLASS_CLUTTER
                        break
                else:
                    # Also label stationary unmapped corridor obstacles away from mapped walls as CLUTTER
                    if tr.last_cluster.wall_edt_mean > 0.25 and tr.last_cluster.chord_width < 0.90 and float(feat20[15]) < 0.18:
                        lbl = CLASS_CLUTTER

            edt_t, gx_t, gy_t = self.map_ctx.query_edt_and_grad(cx, cy)
            frame_samples.append(
                {
                    "track_id": tr.track_id,
                    "scenario_idx": sc_idx,
                    "t_sec": now_sec,
                    "x_state": tr.x.astype(np.float32).copy(),
                    "edt_grad": (edt_t, gx_t, gy_t),
                    "feat20": feat20.copy(),
                    "profile": _resample_cluster_profile(tr.last_cluster.points_local, 24),
                    "label": lbl,
                }
            )

        self.frames.append({"t_sec": now_sec, "scenario_idx": sc_idx, "samples": frame_samples})

    def _export_live_dataset(self) -> int:
        """Assemble classification features + 2.0s future trajectories from captured Gazebo frames."""
        clf_features, clf_profiles, clf_labels, clf_ep_ids = [], [], [], []
        traj_seq_inputs, traj_gt_future_xy, traj_kf_vel, traj_ep_ids = [], [], [], []

        horizon = 20
        dt = 0.10
        track_histories: Dict[Tuple[int, int], List[Dict]] = {}

        for fr in self.frames:
            for smp in fr["samples"]:
                clf_features.append(smp["feat20"])
                clf_profiles.append(smp["profile"])
                clf_labels.append(smp["label"])
                clf_ep_ids.append(smp["scenario_idx"])
                if smp["label"] == CLASS_PERSON:
                    key = (smp["scenario_idx"], smp["track_id"])
                    track_histories.setdefault(key, []).append(smp)

        # Build self-supervised 2.0s future trajectory sequences interpolated at exact dt=0.10s steps
        for (sc_idx, _tid), hist in track_histories.items():
            if len(hist) < 8 or (hist[-1]["t_sec"] - hist[0]["t_sec"]) < 2.4:
                continue
            gt_pts = self.person_gt_by_scenario.get(sc_idx, [])
            gt_arr = np.asarray(gt_pts, dtype=np.float64) if gt_pts else None
            gt_has_motion = bool(gt_arr is not None and len(gt_arr) >= 5 and np.max(np.ptp(gt_arr[:, 1:3], axis=0)) > 0.12)
            hist_t = np.array([h["t_sec"] for h in hist], dtype=np.float64)
            hist_xy = np.array([h["x_state"][:2] for h in hist], dtype=np.float64)

            for idx in range(4, len(hist)):
                cur = hist[idx]
                t0 = cur["t_sec"]
                t_end = t0 + horizon * dt
                if t_end > hist_t[-1] + 0.15:
                    continue
                sub = hist[max(0, idx - 9) : idx + 1]
                vx_now, vy_now = float(cur["x_state"][2]), float(cur["x_state"][3])
                spd_now = math.hypot(vx_now, vy_now)
                if spd_now >= 0.10:
                    psi = math.atan2(vy_now, vx_now)
                else:
                    disp = cur["x_state"][:2] - sub[0]["x_state"][:2]
                    psi = math.atan2(float(disp[1]), float(disp[0])) if float(np.linalg.norm(disp)) > 0.05 else 0.0
                cp, sp = math.cos(-psi), math.sin(-psi)

                seq_aligned = []
                for k_i, node_s in enumerate(sub):
                    prev_s = sub[k_i - 1] if k_i > 0 else node_s
                    dt_s = max(0.05, node_s["t_sec"] - prev_s["t_sec"]) if k_i > 0 else dt
                    dp = (node_s["x_state"][:2] - prev_s["x_state"][:2]) * (dt / dt_s)
                    acc = (node_s["x_state"][2:4] - prev_s["x_state"][2:4]) / dt_s
                    edt_t, gx_t, gy_t = node_s["edt_grad"]
                    vx_i, vy_i = float(node_s["x_state"][2]), float(node_s["x_state"][3])
                    seq_aligned.append(
                        [
                            float(dp[0]) * cp - float(dp[1]) * sp,
                            float(dp[0]) * sp + float(dp[1]) * cp,
                            vx_i * cp - vy_i * sp,
                            vx_i * sp + vy_i * cp,
                            float(acc[0]) * cp - float(acc[1]) * sp,
                            float(acc[0]) * sp + float(acc[1]) * cp,
                            edt_t,
                            gx_t * cp - gy_t * sp,
                            gx_t * sp + gy_t * cp,
                            math.hypot(vx_i, vy_i),
                        ]
                    )
                while len(seq_aligned) < 10:
                    seq_aligned.insert(0, [0.0] * 10)

                cur_xy = cur["x_state"][:2]
                use_gt = False
                if gt_has_motion:
                    i0 = int(np.argmin(np.abs(gt_arr[:, 0] - t0)))
                    if abs(gt_arr[i0, 0] - t0) < 0.25 and math.hypot(gt_arr[i0, 1] - float(cur_xy[0]), gt_arr[i0, 2] - float(cur_xy[1])) < 0.65:
                        use_gt = gt_arr[-1, 0] >= t_end - 0.15

                fut_xy = []
                for step_k in range(1, horizon + 1):
                    t_target = t0 + step_k * dt
                    if use_gt:
                        fx = float(np.interp(t_target, gt_arr[:, 0], gt_arr[:, 1]))
                        fy = float(np.interp(t_target, gt_arr[:, 0], gt_arr[:, 2]))
                        p_fut = np.array([fx, fy], dtype=np.float32)
                    else:
                        fx = float(np.interp(t_target, hist_t, hist_xy[:, 0]))
                        fy = float(np.interp(t_target, hist_t, hist_xy[:, 1]))
                        p_fut = np.array([fx, fy], dtype=np.float32)
                    delta = p_fut - cur_xy
                    fut_xy.append([float(delta[0]) * cp - float(delta[1]) * sp, float(delta[0]) * sp + float(delta[1]) * cp])

                kf_v_l = np.array([vx_now * cp - vy_now * sp, vx_now * sp + vy_now * cp], dtype=np.float32)
                traj_seq_inputs.append(np.asarray(seq_aligned, dtype=np.float32))
                traj_gt_future_xy.append(np.asarray(fut_xy, dtype=np.float32))
                traj_kf_vel.append(kf_v_l)
                traj_ep_ids.append(sc_idx)

        np.savez_compressed(
            self.sim_dataset_path,
            clf_features=np.asarray(clf_features, dtype=np.float32),
            clf_profiles=np.asarray(clf_profiles, dtype=np.float32),
            clf_labels=np.asarray(clf_labels, dtype=np.int64),
            clf_ep_ids=np.asarray(clf_ep_ids, dtype=np.int64),
            traj_seq_inputs=np.asarray(traj_seq_inputs, dtype=np.float32),
            traj_gt_future_xy=np.asarray(traj_gt_future_xy, dtype=np.float32),
            traj_kf_vel=np.asarray(traj_kf_vel, dtype=np.float32),
            traj_ep_ids=np.asarray(traj_ep_ids, dtype=np.int64),
        )
        self.get_logger().info(
            f"[AGENT_3: COLLECTOR_TRAINER] Exported {len(self.frames)} live Gazebo frames -> "
            f"{len(clf_labels)} cluster samples (WALL={clf_labels.count(0)}, CLUTTER={clf_labels.count(1)}, PERSON={clf_labels.count(2)}), "
            f"{len(traj_seq_inputs)} 2.0s trajectory sequences -> {self.sim_dataset_path}"
        )
        return len(clf_labels)

    def _run_training_pipeline(self):
        venv_py = os.path.join(self.ws_root, ".venv_ml", "bin", "python")
        py_bin = venv_py if os.path.exists(venv_py) else sys.executable
        train_script = os.path.join(self.ws_root, "src", "agv_navigation", "ml_training", "train_all_models.py")
        env = os.environ.copy()
        env["PYTHONPATH"] = os.path.join(self.ws_root, "src", "agv_navigation") + ":" + os.path.join(self.ws_root, "src", "agv_navigation", "ml_training")
        env["PYTHONUNBUFFERED"] = "1"

        self.get_logger().info("[AGENT_3: COLLECTOR_TRAINER] Launching 80% Sim + 20% Hardware multi-model training & best-model selection...")
        proc = subprocess.Popen(
            [
                py_bin,
                "-u",
                train_script,
                "--ws-root", self.ws_root,
                "--sim-live", self.sim_dataset_path,
            ],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        tail_lines = []
        if proc.stdout is not None:
            for raw_line in proc.stdout:
                line = raw_line.rstrip("\r\n")
                if line:
                    self.get_logger().info(f"[TRAINER] {line}")
                    tail_lines.append(line)
                    if len(tail_lines) > 30:
                        tail_lines.pop(0)
            proc.stdout.close()
        rc = proc.wait()
        if rc != 0:
            self.get_logger().error(
                f"[TRAINER_ERR] exit={rc} tail=" + " | ".join(tail_lines[-10:])
            )
        else:
            r_msg = String()
            r_msg.data = "agent3_auto_trained_best_models"
            self.reload_pub.publish(r_msg)
            self.get_logger().info("[AGENT_3: COLLECTOR_TRAINER] Best models selected, exported, and hot-reload triggered!")
        self.training_finished = True

    def _monitor_tick(self):
        now = time.monotonic()
        elapsed = now - self.start_wall

        n_samples = sum(len(fr["samples"]) for fr in self.frames)
        status = {
            "frames": len(self.frames),
            "samples": n_samples,
            "elapsed_sec": round(elapsed, 1),
            "training_started": self.training_started,
            "training_finished": self.training_finished,
        }
        sm = String()
        sm.data = json.dumps(status)
        self.status_pub.publish(sm)

        if now - self.last_log_wall >= 4.0 and not self.training_started:
            self.last_log_wall = now
            self.get_logger().info(
                f"[AGENT_3: COLLECTOR_TRAINER] Live Capture: {len(self.frames)} Gazebo scans, {n_samples} labeled clusters "
                f"({elapsed:.0f}s/{self.collect_duration_sec:.0f}s)"
            )

        ready_to_train = (
            (self.mission_done and len(self.frames) >= self.min_sim_frames)
            or (elapsed >= self.collect_duration_sec and len(self.frames) >= 40)
        )
        if ready_to_train and not self.training_started:
            self.training_started = True
            self._export_live_dataset()
            if self.auto_train:
                self._run_training_pipeline()


def main(args=None):
    rclpy.init(args=args)
    node = DatasetTrainerAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.training_started and len(node.frames) > 0:
            node._export_live_dataset()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
