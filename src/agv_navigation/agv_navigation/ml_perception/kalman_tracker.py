"""2D Kalman Multi-Object Tracker with Hungarian assignment and temporal kinematic features."""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from .lidar_clustering import CLASS_CLUTTER, CLASS_NAMES, CLASS_PERSON, CLASS_WALL, ClusterCandidate


@dataclass
class TrackState:
    """Internal persistent Kalman + sliding-window state for one tracked obstacle."""

    track_id: int
    x: np.ndarray                             # [px, py, vx, vy]
    P: np.ndarray                             # (4, 4) covariance
    start_pos: np.ndarray                     # (2,)
    radius: float
    age: int = 1
    misses: int = 0
    pos_history: Deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=12))
    raw_vel_history: Deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=10))
    kf_vel_history: Deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=10))
    width_history: Deque[float] = field(default_factory=lambda: deque(maxlen=10))
    dt_history: Deque[float] = field(default_factory=lambda: deque(maxlen=10))
    class_probs: np.ndarray = field(default_factory=lambda: np.array([0.2, 0.5, 0.3], dtype=np.float32))
    gru_hidden: Optional[np.ndarray] = None
    last_cluster: Optional[ClusterCandidate] = None


@dataclass
class TrackedObstacle:
    """Public snapshot of a tracked cluster with classification & trajectory prediction."""

    track_id: int
    pos: np.ndarray                           # (2,) map frame [x, y]
    vel: np.ndarray                           # (2,) map frame [vx, vy]
    speed: float
    radius: float
    age: int
    misses: int
    displacement: float
    class_id: int
    class_name: str
    class_probs: np.ndarray                   # [P(WALL), P(CLUTTER), P(PERSON)]
    features: np.ndarray                      # (20,) full feature vector
    is_leg_pair: bool = False
    pred_traj: Optional[np.ndarray] = None    # (T, 2) map frame
    pred_std: Optional[np.ndarray] = None     # (T, 2) uncertainty std (m)


_H_MAT = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
_EYE4 = np.eye(4, dtype=np.float64)


def _inv2x2(s00: float, s01: float, s10: float, s11: float, fallback_inv: float) -> np.ndarray:
    det = s00 * s11 - s01 * s10
    if abs(det) < 1e-12:
        return np.array([[fallback_inv, 0.0], [0.0, fallback_inv]], dtype=np.float64)
    inv_d = 1.0 / det
    return np.array([[s11 * inv_d, -s01 * inv_d], [-s10 * inv_d, s00 * inv_d]], dtype=np.float64)


class KalmanTracker:
    """Constant-velocity 2D Kalman tracker with Mahalanobis-gated Hungarian matching."""

    def __init__(
        self,
        process_accel_std: float = 1.35,
        meas_std: float = 0.06,
        max_assoc_dist: float = 0.60,
        mahalanobis_gate: float = 16.0,
        max_misses: int = 4,
    ):
        self.sigma_a = float(process_accel_std)
        self.sigma_z = float(meas_std)
        self.max_assoc_dist = float(max_assoc_dist)
        self.mahalanobis_gate = float(mahalanobis_gate)
        self.max_misses = int(max_misses)

        self.tracks: Dict[int, TrackState] = {}
        self._next_id = 1
        self._last_dt = -1.0
        self._F = np.eye(4, dtype=np.float64)
        self._Q = np.zeros((4, 4), dtype=np.float64)

    def reset(self):
        self.tracks.clear()
        self._next_id = 1

    def _prepare_fq(self, dt: float):
        if abs(dt - self._last_dt) < 1e-6:
            return
        self._last_dt = dt
        self._F[0, 2] = dt
        self._F[1, 3] = dt
        dt2 = dt * dt
        dt3 = dt2 * dt * 0.5
        dt4 = dt2 * dt2 * 0.25
        sa2 = self.sigma_a * self.sigma_a
        self._Q[0, 0] = self._Q[1, 1] = sa2 * dt4
        self._Q[0, 2] = self._Q[2, 0] = self._Q[1, 3] = self._Q[3, 1] = sa2 * dt3
        self._Q[2, 2] = self._Q[3, 3] = sa2 * dt2

    def _predict_track(self, tr: TrackState, dt: float):
        self._prepare_fq(dt)
        tr.x = self._F @ tr.x
        tr.P = self._F @ tr.P @ self._F.T + self._Q

    def _compute_temporal_features(self, tr: TrackState) -> np.ndarray:
        """Compute the 5 temporal kinematic features: speed, vel_var, net_disp_ratio, width_std, log_age."""
        kf_speed = math.hypot(float(tr.x[2]), float(tr.x[3]))

        n_v = len(tr.raw_vel_history)
        if n_v >= 2:
            vx_s = 0.0
            vy_s = 0.0
            for v in tr.raw_vel_history:
                vx_s += float(v[0])
                vy_s += float(v[1])
            mvx, mvy = vx_s / n_v, vy_s / n_v
            var_s = 0.0
            for v in tr.raw_vel_history:
                dvx = float(v[0]) - mvx
                dvy = float(v[1]) - mvy
                var_s += dvx * dvx + dvy * dvy
            vel_var = min(4.0, var_s / n_v)
        else:
            vel_var = 0.0

        n_p = len(tr.pos_history)
        if n_p >= 2:
            p0 = tr.pos_history[0]
            pl = tr.pos_history[-1]
            net_disp = math.hypot(float(pl[0]) - float(p0[0]), float(pl[1]) - float(p0[1]))
            step_path = 0.0
            prev = p0
            for idx_p in range(1, n_p):
                cur = tr.pos_history[idx_p]
                step_path += math.hypot(float(cur[0]) - float(prev[0]), float(cur[1]) - float(prev[1]))
                prev = cur
            net_ratio = min(1.0, max(0.0, net_disp / (step_path + 1e-3))) if step_path > 1e-3 else 0.0
        else:
            net_ratio = 0.0

        n_w = len(tr.width_history)
        if n_w >= 2:
            mw = sum(tr.width_history) / n_w
            width_std = math.sqrt(sum((w - mw) * (w - mw) for w in tr.width_history) / n_w)
        else:
            width_std = 0.0
        log_age = math.log1p(tr.age)
        return np.array([kf_speed, vel_var, net_ratio, width_std, log_age], dtype=np.float32)

    def update(
        self,
        clusters: List[ClusterCandidate],
        dt: float,
        robot_ang_vel: float = 0.0,
    ) -> List[Tuple[TrackState, np.ndarray]]:
        """Advance all Kalman tracks by dt and associate new cluster measurements.

        Returns list of (TrackState, full_20d_feature_vector) for active matched/coasting tracks.
        """
        dt = float(max(0.04, min(0.35, dt)))

        # Measurement noise scales mildly when robot turns quickly (reduces scan-skew velocity spikes)
        meas_std = self.sigma_z + 0.04 * min(1.5, abs(robot_ang_vel))
        r_var = meas_std * meas_std
        fallback_inv = 1.0 / (r_var + 1e-4)

        track_ids = list(self.tracks.keys())
        for tid in track_ids:
            self._predict_track(self.tracks[tid], dt)

        matched_tracks = set()
        matched_clusters = set()

        if track_ids and clusters:
            n_tr = len(track_ids)
            n_cl = len(clusters)
            BIG_COST = 1e6
            cost = np.full((n_tr, n_cl), BIG_COST, dtype=np.float64)
            s_inv_list = []

            for r_i, tid in enumerate(track_ids):
                tr = self.tracks[tid]
                z_pred = tr.x[:2]
                S_inv = _inv2x2(
                    float(tr.P[0, 0] + r_var), float(tr.P[0, 1]),
                    float(tr.P[1, 0]), float(tr.P[1, 1] + r_var),
                    fallback_inv,
                )
                s_inv_list.append(S_inv)
                inv00, inv01, inv11 = float(S_inv[0, 0]), float(S_inv[0, 1]), float(S_inv[1, 1])

                for c_j, cl in enumerate(clusters):
                    ix = float(cl.centroid_map[0]) - float(z_pred[0])
                    iy = float(cl.centroid_map[1]) - float(z_pred[1])
                    euclid = math.hypot(ix, iy)
                    if euclid > self.max_assoc_dist:
                        continue
                    maha2 = ix * ix * inv00 + 2.0 * ix * iy * inv01 + iy * iy * inv11
                    if maha2 > self.mahalanobis_gate and euclid > 0.30:
                        continue
                    width_pen = 0.5 * abs(cl.chord_width - (tr.width_history[-1] if tr.width_history else cl.chord_width))
                    cost[r_i, c_j] = maha2 + 2.0 * euclid + width_pen

            row_ind, col_ind = linear_sum_assignment(cost)
            for r_i, c_j in zip(row_ind, col_ind):
                if cost[r_i, c_j] >= BIG_COST * 0.5:
                    continue
                tid = track_ids[r_i]
                tr = self.tracks[tid]
                cl = clusters[c_j]
                z = cl.centroid_map.astype(np.float64)

                prev_pos = tr.pos_history[-1] if tr.pos_history else z.astype(np.float32)
                raw_vel = (z.astype(np.float32) - prev_pos) / dt

                # Kalman update using closed-form S_inv
                y = z - tr.x[:2]
                K = tr.P[:, :2] @ s_inv_list[r_i]
                tr.x = tr.x + K @ y
                tr.P = (_EYE4 - K @ _H_MAT) @ tr.P

                tr.age += 1
                tr.misses = 0
                tr.radius = 0.5 * tr.radius + 0.5 * cl.radius_est
                tr.pos_history.append(tr.x[:2].astype(np.float32).copy())
                tr.raw_vel_history.append(raw_vel)
                tr.kf_vel_history.append(tr.x[2:4].astype(np.float32).copy())
                tr.width_history.append(cl.chord_width)
                tr.dt_history.append(dt)
                tr.last_cluster = cl

                matched_tracks.add(tid)
                matched_clusters.add(c_j)

        # Spawn new tracks for unmatched clusters
        for c_j, cl in enumerate(clusters):
            if c_j in matched_clusters:
                continue
            tid = self._next_id
            self._next_id += 1
            pos = cl.centroid_map.astype(np.float64)
            x0 = np.array([pos[0], pos[1], 0.0, 0.0], dtype=np.float64)
            P0 = np.diag([0.04, 0.04, 0.64, 0.64]).astype(np.float64)
            tr = TrackState(
                track_id=tid,
                x=x0,
                P=P0,
                start_pos=pos.astype(np.float32).copy(),
                radius=cl.radius_est,
                age=1,
                misses=0,
                last_cluster=cl,
            )
            tr.pos_history.append(pos.astype(np.float32).copy())
            tr.raw_vel_history.append(np.zeros(2, dtype=np.float32))
            tr.kf_vel_history.append(np.zeros(2, dtype=np.float32))
            tr.width_history.append(cl.chord_width)
            tr.dt_history.append(dt)
            self.tracks[tid] = tr
            matched_tracks.add(tid)

        # Age out unmatched tracks; coast brief occlusions (misses <= max_misses)
        to_delete = []
        for tid in track_ids:
            if tid not in matched_tracks:
                tr = self.tracks[tid]
                tr.misses += 1
                if tr.misses > self.max_misses:
                    to_delete.append(tid)
                else:
                    tr.pos_history.append(tr.x[:2].astype(np.float32).copy())
                    tr.kf_vel_history.append(tr.x[2:4].astype(np.float32).copy())
                    tr.dt_history.append(dt)
        for tid in to_delete:
            del self.tracks[tid]

        # Build 20-D feature vector per active track
        active: List[Tuple[TrackState, np.ndarray]] = []
        for tr in self.tracks.values():
            if tr.last_cluster is None:
                continue
            temp_feats = self._compute_temporal_features(tr)
            full_feats = np.concatenate((tr.last_cluster.geom_features, temp_feats))
            active.append((tr, full_feats))
        return active
