"""Unified 2D LiDAR ML perception pipeline combining clustering, Kalman tracking, classification, and prediction."""

import math
from typing import List, Optional, Tuple

import numpy as np

from .classifier_runtime import ObstacleClassifierRuntime
from .kalman_tracker import KalmanTracker, TrackedObstacle
from .lidar_clustering import (
    CLASS_CLUTTER,
    CLASS_NAMES,
    CLASS_PERSON,
    CLASS_WALL,
    MapContext,
    extract_lidar_clusters,
)
from .predictor_runtime import TrajectoryPredictorRuntime


class PerceptionPipeline:
    """End-to-end 10 Hz scan processor producing classified & predicted obstacle tracks."""

    def __init__(
        self,
        classifier_path: Optional[str] = None,
        predictor_path: Optional[str] = None,
        prediction_steps: int = 20,
        dt: float = 0.10,
        enable_ml: bool = True,
    ):
        self.enable_ml = bool(enable_ml)
        self.prediction_steps = int(prediction_steps)
        self.dt = float(dt)

        self.map_ctx = MapContext()
        self.tracker = KalmanTracker()
        self.classifier = ObstacleClassifierRuntime(classifier_path if enable_ml else None)
        self.predictor = TrajectoryPredictorRuntime(
            predictor_path if enable_ml else None,
            default_horizon=prediction_steps,
            dt=dt,
        )

    def update_map(self, occ_grid: np.ndarray, resolution: float, origin_x: float, origin_y: float):
        """Update static occupancy grid used for wall EDT and predictor context."""
        self.map_ctx.update(occ_grid, resolution, origin_x, origin_y)

    def reset(self):
        self.tracker.reset()
        self.predictor.reset()

    def process_scan(
        self,
        ranges: np.ndarray,
        angles: np.ndarray,
        pose_x: float,
        pose_y: float,
        pose_yaw: float,
        dt: float,
        range_min: float = 0.15,
        range_max: float = 8.0,
        robot_ang_vel: float = 0.0,
        prediction_steps: Optional[int] = None,
    ) -> Tuple[List[TrackedObstacle], np.ndarray, np.ndarray]:
        """Process a raw 2D LaserScan into classified tracks with 1-2s future predictions.

        Returns:
            (tracked_obstacles, all_external_local_pts, all_external_map_pts)
        """
        clusters, ext_local, ext_map = extract_lidar_clusters(
            ranges=ranges,
            angles=angles,
            pose_x=pose_x,
            pose_y=pose_y,
            pose_yaw=pose_yaw,
            range_min=range_min,
            range_max=range_max,
            map_ctx=self.map_ctx if self.map_ctx.edt_grid is not None else None,
        )

        active = self.tracker.update(clusters, dt=dt, robot_ang_vel=robot_ang_vel)
        if not active:
            self.predictor.prune_stale(set())
            return [], ext_local, ext_map

        feat_batch = np.stack([feats for _, feats in active], axis=0)
        obs_probs_batch = self.classifier.predict_proba_batch(feat_batch)

        steps = int(prediction_steps) if prediction_steps is not None else self.prediction_steps
        active_ids = set()
        results: List[TrackedObstacle] = []

        for (tr, feats), obs_probs in zip(active, obs_probs_batch):
            active_ids.add(tr.track_id)
            if tr.age == 1:
                tr.class_probs = obs_probs.copy()
            elif tr.misses == 0:
                tr.class_probs = self.classifier.bayesian_filter(tr.class_probs, obs_probs, alpha=0.38)

            # Safety guard: stationary long wall segments touching static wall map remain WALL
            # even under sliding-aperture centroid motion
            if tr.last_cluster is not None:
                if tr.last_cluster.wall_edt_mean < 0.12 and tr.last_cluster.chord_width > 0.90 and float(feats[7]) > 0.94:
                    tr.class_probs = np.array([0.92, 0.06, 0.02], dtype=np.float32)

            cls_id = int(np.argmax(tr.class_probs))
            pos = tr.x[:2].astype(np.float32).copy()
            vel = tr.x[2:4].astype(np.float32).copy()
            speed = float(math.hypot(float(vel[0]), float(vel[1])))
            disp = float(np.linalg.norm(pos - tr.start_pos))

            self.predictor.update_history(tr, dt=dt, map_ctx=self.map_ctx if self.map_ctx.edt_grid is not None else None)

            pred_traj = None
            pred_std = None
            if cls_id == CLASS_PERSON:
                pred_traj, pred_std = self.predictor.predict(tr, steps=steps)

            results.append(
                TrackedObstacle(
                    track_id=tr.track_id,
                    pos=pos,
                    vel=vel,
                    speed=speed,
                    radius=float(tr.radius),
                    age=tr.age,
                    misses=tr.misses,
                    displacement=disp,
                    class_id=cls_id,
                    class_name=CLASS_NAMES[cls_id],
                    class_probs=tr.class_probs.copy(),
                    features=feats,
                    is_leg_pair=bool(tr.last_cluster.is_leg_pair) if tr.last_cluster else False,
                    pred_traj=pred_traj,
                    pred_std=pred_std,
                )
            )

        self.predictor.prune_stale(active_ids)
        return results, ext_local, ext_map
