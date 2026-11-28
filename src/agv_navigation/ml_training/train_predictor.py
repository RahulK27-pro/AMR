"""Train & benchmark short-horizon (1.0-2.0s) Kalman-NN (Residual GRU + Uncertainty) predictor.

Compares against Constant-Velocity Kalman Filter and Pure-GRU baselines on ADE, FDE,
and Gaussian NLL calibration, exporting weights to pure-NumPy .npz for runtime execution.
"""

import argparse
import json
import math
import os
from typing import Tuple

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset


class KalmanResidualGRU(nn.Module):
    """1-layer GRUCell encoder over 10-step track-aligned history + residual velocity decoder."""

    def __init__(self, in_dim: int = 10, hidden_dim: int = 32, horizon: int = 20, dt: float = 0.1):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.horizon = horizon
        self.dt = dt
        self.gru_cell = nn.GRUCell(in_dim, hidden_dim)
        self.dec_fc1 = nn.Linear(hidden_dim, 64)
        self.dec_fc2 = nn.Linear(64, horizon * 4)

    def forward(self, seq: torch.Tensor, kf_vel_local: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Predict future (B, T, 2) positions and (B, T, 2) stds in track-aligned frame."""
        bsz, seq_len, _ = seq.shape
        h = torch.zeros(bsz, self.hidden_dim, dtype=seq.dtype, device=seq.device)
        for t in range(seq_len):
            h = self.gru_cell(seq[:, t, :], h)

        dec = torch.relu(self.dec_fc1(h))
        out = self.dec_fc2(dec).view(bsz, self.horizon, 4)

        dv = 0.48 * torch.tanh(out[:, :, :2])
        # Physics-bounded velocity residual around KF prior (prevents OOD wall-distance collapse)
        pred_vel = kf_vel_local.unsqueeze(1) + dv
        pred_pos = torch.cumsum(pred_vel * self.dt, dim=1)
        pred_std = torch.nn.functional.softplus(out[:, :, 2:4]) + 0.04
        return pred_pos, pred_std


class PureGRU(nn.Module):
    """Pure GRU ablation without Kalman velocity prior."""

    def __init__(self, in_dim: int = 10, hidden_dim: int = 32, horizon: int = 20, dt: float = 0.1):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.horizon = horizon
        self.dt = dt
        self.gru_cell = nn.GRUCell(in_dim, hidden_dim)
        self.head = nn.Sequential(nn.Linear(hidden_dim, 64), nn.ReLU(), nn.Linear(64, horizon * 2))

    def forward(self, seq: torch.Tensor) -> torch.Tensor:
        bsz, seq_len, _ = seq.shape
        h = torch.zeros(bsz, self.hidden_dim, dtype=seq.dtype, device=seq.device)
        for t in range(seq_len):
            h = self.gru_cell(seq[:, t, :], h)
        vel = self.head(h).view(bsz, self.horizon, 2)
        return torch.cumsum(vel * self.dt, dim=1)


def _episode_split(ep_ids: np.ndarray, val_frac: float = 0.20, seed: int = 42) -> Tuple[np.ndarray, np.ndarray]:
    unique_eps = np.unique(ep_ids)
    rng = np.random.default_rng(seed)
    rng.shuffle(unique_eps)
    n_val = max(1, int(len(unique_eps) * val_frac))
    val_set = set(unique_eps[:n_val].tolist())
    val_mask = np.array([int(e) in val_set for e in ep_ids], dtype=bool)
    return ~val_mask, val_mask


def _metrics(pred: np.ndarray, gt: np.ndarray) -> dict:
    errs = np.linalg.norm(pred - gt, axis=-1)  # (N, T)
    return {
        "ade_1s": round(float(np.mean(errs[:, :10])), 4),
        "fde_1s": round(float(np.mean(errs[:, 9])), 4),
        "ade_2s": round(float(np.mean(errs)), 4),
        "fde_2s": round(float(np.mean(errs[:, -1])), 4),
    }


def train_and_benchmark(dataset_path: str, model_out: str, meta_out: str, epochs: int = 45):
    data = np.load(dataset_path)
    seq = data["traj_seq_inputs"].astype(np.float32)       # (N, 10, 10)
    gt = data["traj_gt_future_xy"].astype(np.float32)      # (N, 20, 2)
    kf_vel = data["traj_kf_vel"].astype(np.float32)        # (N, 2)
    ep_ids = data["traj_ep_ids"] if "traj_ep_ids" in data else np.arange(len(seq)) // 50

    train_mask, val_mask = _episode_split(ep_ids, val_frac=0.20, seed=42)
    seq_tr, seq_va = seq[train_mask], seq[val_mask]
    gt_tr, gt_va = gt[train_mask], gt[val_mask]
    kv_tr, kv_va = kf_vel[train_mask], kf_vel[val_mask]

    # Standardize non-zero entries across sequence features
    flat_tr = seq_tr.reshape(-1, seq_tr.shape[-1])
    mean = np.mean(flat_tr, axis=0)
    std = np.maximum(np.std(flat_tr, axis=0), 1e-4)
    seq_tr_n = (seq_tr - mean) / std
    seq_va_n = (seq_va - mean) / std

    dt = 0.10
    horizon = gt.shape[1]
    t_steps = (np.arange(1, horizon + 1, dtype=np.float32) * dt)[None, :, None]

    # 1. Constant-Velocity Kalman baseline
    cv_pred_va = kv_va[:, None, :] * t_steps
    cv_metrics = _metrics(cv_pred_va, gt_va)
    print(f"[PRED] Constant-Velocity KF baseline: {cv_metrics}")

    # 2. Pure-GRU ablation
    torch.manual_seed(42)
    pure_gru = PureGRU(in_dim=seq.shape[-1], hidden_dim=32, horizon=horizon, dt=dt)
    opt_p = torch.optim.AdamW(pure_gru.parameters(), lr=3e-3, weight_decay=2e-4)
    dl_p = DataLoader(TensorDataset(torch.from_numpy(seq_tr_n), torch.from_numpy(gt_tr)), batch_size=256, shuffle=True)
    for _ in range(min(epochs, 30)):
        pure_gru.train()
        for bs, bg in dl_p:
            opt_p.zero_grad()
            loss = torch.nn.functional.smooth_l1_loss(pure_gru(bs), bg, beta=0.15)
            loss.backward()
            opt_p.step()
    pure_gru.eval()
    with torch.no_grad():
        pure_pred_va = pure_gru(torch.from_numpy(seq_va_n)).numpy()
    pure_metrics = _metrics(pure_pred_va, gt_va)
    print(f"[PRED] Pure-GRU baseline:             {pure_metrics}")

    # 3. Production Kalman-NN (Residual GRU + Heteroscedastic Uncertainty)
    torch.manual_seed(42)
    model = KalmanResidualGRU(in_dim=seq.shape[-1], hidden_dim=32, horizon=horizon, dt=dt)
    opt = torch.optim.AdamW(model.parameters(), lr=3e-3, weight_decay=3e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    dl = DataLoader(
        TensorDataset(torch.from_numpy(seq_tr_n), torch.from_numpy(kv_tr), torch.from_numpy(gt_tr)),
        batch_size=256,
        shuffle=True,
    )

    best_score = float("inf")
    best_state = None
    for ep in range(epochs):
        model.train()
        for bs, bkv, bgt in dl:
            opt.zero_grad()
            pred_pos, pred_std = model(bs, bkv)
            # Combined Huber trajectory loss + calibrated Gaussian NLL
            huber = torch.nn.functional.smooth_l1_loss(pred_pos, bgt, beta=0.15)
            nll = 0.5 * torch.mean(((bgt - pred_pos) / pred_std) ** 2 + 2.0 * torch.log(pred_std))
            loss = huber + 0.15 * nll
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 2.0)
            opt.step()
        sched.step()

        model.eval()
        with torch.no_grad():
            va_pos, va_std = model(torch.from_numpy(seq_va_n), torch.from_numpy(kv_va))
        va_pos_np = va_pos.numpy()
        ade = float(np.mean(np.linalg.norm(va_pos_np - gt_va, axis=-1)))
        if ade < best_score:
            best_score = ade
            best_state = {k: v.detach().cpu().numpy().copy() for k, v in model.state_dict().items()}

    model.load_state_dict({k: torch.from_numpy(v) for k, v in best_state.items()})
    model.eval()
    with torch.no_grad():
        va_pos, va_std = model(torch.from_numpy(seq_va_n), torch.from_numpy(kv_va))
    va_pos_np = va_pos.numpy()
    va_std_np = va_std.numpy()
    knn_metrics = _metrics(va_pos_np, gt_va)
    # 95% coverage check: |err_x| <= 1.96*std_x and |err_y| <= 1.96*std_y
    cov_95 = float(np.mean(np.abs(gt_va - va_pos_np) <= 1.96 * va_std_np))
    knn_metrics["cov_95_axis"] = round(cov_95, 4)
    knn_metrics["fde_2s_improvement_vs_kf_pct"] = round(
        100.0 * (cv_metrics["fde_2s"] - knn_metrics["fde_2s"]) / max(1e-4, cv_metrics["fde_2s"]), 2
    )
    print(f"[PRED] Kalman-NN (Residual GRU):      {knn_metrics}")

    os.makedirs(os.path.dirname(model_out), exist_ok=True)
    np.savez(
        model_out,
        mean=mean.astype(np.float32),
        std=std.astype(np.float32),
        dt=np.float32(dt),
        horizon=np.int32(horizon),
        hidden_dim=np.int32(32),
        w_ih=best_state["gru_cell.weight_ih"].astype(np.float32),
        b_ih=best_state["gru_cell.bias_ih"].astype(np.float32),
        w_hh=best_state["gru_cell.weight_hh"].astype(np.float32),
        b_hh=best_state["gru_cell.bias_hh"].astype(np.float32),
        dec1_w=best_state["dec_fc1.weight"].astype(np.float32),
        dec1_b=best_state["dec_fc1.bias"].astype(np.float32),
        dec2_w=best_state["dec_fc2.weight"].astype(np.float32),
        dec2_b=best_state["dec_fc2.bias"].astype(np.float32),
    )

    meta = {}
    if os.path.exists(meta_out):
        try:
            with open(meta_out, "r") as f:
                meta = json.load(f)
        except Exception:
            meta = {}
    meta["predictor"] = {
        "horizon_steps": int(horizon),
        "dt": float(dt),
        "hidden_dim": 32,
        "benchmarks": {
            "constant_velocity_kf": cv_metrics,
            "pure_gru": pure_metrics,
            "kalman_nn_residual_gru": knn_metrics,
        },
        "num_train": int(len(seq_tr)),
        "num_val": int(len(seq_va)),
    }
    with open(meta_out, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[PRED] Exported pure-NumPy Kalman-NN weights -> {model_out}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default=os.path.expanduser("~/AMR/AMR-main/ml_datasets/synthetic_lidar_dataset.npz"))
    parser.add_argument("--out", default=os.path.expanduser("~/AMR/AMR-main/src/agv_navigation/models/trajectory_predictor.npz"))
    parser.add_argument("--meta", default=os.path.expanduser("~/AMR/AMR-main/src/agv_navigation/models/model_metadata.json"))
    parser.add_argument("--epochs", type=int, default=45)
    args = parser.parse_args()
    train_and_benchmark(args.dataset, args.out, args.meta, args.epochs)


if __name__ == "__main__":
    main()
