"""Pure-NumPy runtime Kalman-NN (Residual GRU + Uncertainty) short-horizon trajectory predictor."""

from collections import deque
import math
import os
from typing import Deque, Dict, Optional, Tuple

import numpy as np

from .kalman_tracker import TrackState
from .lidar_clustering import MapContext


def _sigmoid(x: np.ndarray) -> np.ndarray:
    x_clip = np.clip(x, -25.0, 25.0)
    return 1.0 / (1.0 + np.exp(-x_clip))


def _softplus(x: np.ndarray) -> np.ndarray:
    return np.where(x > 20.0, x, np.log1p(np.exp(np.clip(x, -25.0, 20.0))))


class TrajectoryPredictorRuntime:
    """Predicts 1-2s future positions and uncertainty bounds using KF prior + Residual GRU."""

    def __init__(self, model_path: Optional[str] = None, default_horizon: int = 20, dt: float = 0.10):
        self.loaded = False
        self.horizon = int(default_horizon)
        self.dt = float(dt)
        self.hidden_dim = 32
        self.weights: Dict[str, np.ndarray] = {}
        # Raw map-frame kinematic buffer per track_id: list of 9-dim vectors
        self._raw_hist: Dict[int, Deque[Tuple[np.ndarray, np.ndarray]]] = {}
        if model_path and os.path.exists(model_path):
            self.load(model_path)

    def load(self, model_path: str):
        data = np.load(model_path)
        self.weights = {k: data[k].astype(np.float32) for k in data.files if k not in ("horizon", "hidden_dim", "dt")}
        if "horizon" in data:
            self.horizon = int(data["horizon"])
        if "hidden_dim" in data:
            self.hidden_dim = int(data["hidden_dim"])
        if "dt" in data:
            self.dt = float(data["dt"])
        self.loaded = True

    def reset(self):
        self._raw_hist.clear()

    def prune_stale(self, active_ids):
        stale = [tid for tid in self._raw_hist if tid not in active_ids]
        for tid in stale:
            del self._raw_hist[tid]

    def update_history(self, tr: TrackState, dt: float, map_ctx: Optional[MapContext]):
        """Append latest Kalman state & static wall EDT/gradient to track sequence buffer."""
        hist = self._raw_hist.setdefault(tr.track_id, deque(maxlen=10))
        px, py = float(tr.x[0]), float(tr.x[1])
        vx, vy = float(tr.x[2]), float(tr.x[3])
        if hist:
            prev_x, _ = hist[-1]
            dp = tr.x[:2].astype(np.float32) - prev_x[:2]
            acc = (tr.x[2:4].astype(np.float32) - prev_x[2:4]) / max(0.04, dt)
        else:
            dp = np.zeros(2, dtype=np.float32)
            acc = np.zeros(2, dtype=np.float32)

        if map_ctx is not None:
            edt_t, gx_t, gy_t = map_ctx.query_edt_and_grad(px, py)
        else:
            edt_t, gx_t, gy_t = 3.0, 0.0, 0.0

        raw_vec = np.array([dp[0], dp[1], vx, vy, acc[0], acc[1], edt_t, gx_t, gy_t], dtype=np.float32)
        hist.append((tr.x.astype(np.float32).copy(), raw_vec))

    def _run_gru_sequence(self, seq_norm: np.ndarray) -> np.ndarray:
        """Exact PyTorch GRUCell unroll in pure NumPy over (seq_len, 10) -> hidden (H,)."""
        w = self.weights
        H = self.hidden_dim
        h = np.zeros(H, dtype=np.float32)
        gi_all = seq_norm @ w["w_ih"].T + w["b_ih"]
        w_hh_t, b_hh = w["w_hh"].T, w["b_hh"]
        h2 = 2 * H
        for t in range(len(seq_norm)):
            gi = gi_all[t]
            gh = h @ w_hh_t + b_hh
            r = _sigmoid(gi[:H] + gh[:H])
            z = _sigmoid(gi[H:h2] + gh[H:h2])
            n = np.tanh(gi[h2:] + r * gh[h2:])
            h = (1.0 - z) * n + z * h
        return h

    def predict(
        self,
        tr: TrackState,
        steps: Optional[int] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Predict future (T, 2) positions and (T, 2) uncertainty stds in map frame."""
        T = int(steps) if steps is not None else self.horizon
        px, py = float(tr.x[0]), float(tr.x[1])
        vx, vy = float(tr.x[2]), float(tr.x[3])

        hist = self._raw_hist.get(tr.track_id)
        if not self.loaded or hist is None or len(hist) < 4:
            # Immediate physics-grounded Kalman CV fallback for young tracks
            t_arr = (np.arange(1, T + 1, dtype=np.float32) * self.dt)[:, None]
            pred_xy = np.array([px, py], dtype=np.float32)[None, :] + np.array([vx, vy], dtype=np.float32)[None, :] * t_arr
            # Uncertainty grows linearly with prediction time + initial covariance
            base_std = np.sqrt(np.maximum(0.01, np.array([tr.P[0, 0], tr.P[1, 1]], dtype=np.float32)))
            pred_std = base_std[None, :] + 0.14 * t_arr
            return pred_xy.astype(np.float32), pred_std.astype(np.float32)

        spd_now = math.hypot(vx, vy)
        if spd_now >= 0.10:
            psi = math.atan2(vy, vx)
        else:
            disp_vec = tr.x[:2].astype(np.float32) - hist[0][0][:2]
            psi = math.atan2(float(disp_vec[1]), float(disp_vec[0])) if float(np.linalg.norm(disp_vec)) > 0.05 else 0.0

        cm, sm = math.cos(-psi), math.sin(-psi)
        raw_arr = np.stack([rv for _, rv in hist], axis=0)  # (K, 9)
        k_len = len(raw_arr)
        seq_arr = np.zeros((10, 10), dtype=np.float32)
        offset = 10 - k_len
        seq_arr[offset:, 0] = raw_arr[:, 0] * cm - raw_arr[:, 1] * sm
        seq_arr[offset:, 1] = raw_arr[:, 0] * sm + raw_arr[:, 1] * cm
        vx_l = raw_arr[:, 2] * cm - raw_arr[:, 3] * sm
        vy_l = raw_arr[:, 2] * sm + raw_arr[:, 3] * cm
        seq_arr[offset:, 2] = vx_l
        seq_arr[offset:, 3] = vy_l
        seq_arr[offset:, 4] = raw_arr[:, 4] * cm - raw_arr[:, 5] * sm
        seq_arr[offset:, 5] = raw_arr[:, 4] * sm + raw_arr[:, 5] * cm
        seq_arr[offset:, 6] = raw_arr[:, 6]
        seq_arr[offset:, 7] = raw_arr[:, 7] * cm - raw_arr[:, 8] * sm
        seq_arr[offset:, 8] = raw_arr[:, 7] * sm + raw_arr[:, 8] * cm
        seq_arr[offset:, 9] = np.hypot(vx_l, vy_l)

        w = self.weights
        seq_norm = (seq_arr - w["mean"]) / w["std"]
        h = self._run_gru_sequence(seq_norm)

        dec = np.maximum(0.0, h @ w["dec1_w"].T + w["dec1_b"])
        out = (dec @ w["dec2_w"].T + w["dec2_b"]).reshape(self.horizon, 4)

        kf_v_local = np.array([vx * cm - vy * sm, vx * sm + vy * cm], dtype=np.float32)
        pred_v_local = kf_v_local[None, :] + 0.48 * np.tanh(out[:, :2])
        pred_pos_local = np.cumsum(pred_v_local * self.dt, axis=0)
        std_local = _softplus(out[:, 2:4]) + 0.04

        # Rotate predicted local trajectory back to map frame
        cp, sp = math.cos(psi), math.sin(psi)
        pred_map = np.column_stack(
            (
                px + pred_pos_local[:, 0] * cp - pred_pos_local[:, 1] * sp,
                py + pred_pos_local[:, 0] * sp + pred_pos_local[:, 1] * cp,
            )
        ).astype(np.float32)

        # Propagate rotated uncertainty envelope std_x, std_y in map axes
        var_x = (cp * std_local[:, 0]) ** 2 + (sp * std_local[:, 1]) ** 2
        var_y = (sp * std_local[:, 0]) ** 2 + (cp * std_local[:, 1]) ** 2
        std_map = np.sqrt(np.column_stack((var_x, var_y))).astype(np.float32)

        if T < len(pred_map):
            return pred_map[:T], std_map[:T]
        if T > len(pred_map):
            extra = T - len(pred_map)
            last_v = (pred_map[-1] - pred_map[-2]) if len(pred_map) >= 2 else np.array([vx * self.dt, vy * self.dt], dtype=np.float32)
            ext_p = [pred_map[-1] + (k + 1) * last_v for k in range(extra)]
            ext_s = [std_map[-1] + (k + 1) * 0.015 for k in range(extra)]
            pred_map = np.vstack((pred_map, np.asarray(ext_p, dtype=np.float32)))
            std_map = np.vstack((std_map, np.asarray(ext_s, dtype=np.float32)))
        return pred_map, std_map
