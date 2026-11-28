"""Train & benchmark Random Forest, 1D-CNN, and Compact MLP obstacle classifiers.

Exports the calibrated Compact MLP (20->64->32->3) to pure-NumPy .npz weights
for zero-dependency real-time ROS 2 execution on Laptop and Raspberry Pi 5.
"""

import argparse
import json
import os
import time
from typing import Dict, Tuple

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix, f1_score
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from agv_navigation.ml_perception.lidar_clustering import CLASS_NAMES, FEATURE_NAMES


class CompactMLP(nn.Module):
    """20 -> 96 -> 48 -> 3 classifier with LayerNorm for deterministic pure-NumPy export."""

    def __init__(self, in_dim: int = 20, num_classes: int = 3):
        super().__init__()
        self.fc1 = nn.Linear(in_dim, 96)
        self.ln1 = nn.LayerNorm(96)
        self.fc2 = nn.Linear(96, 48)
        self.ln2 = nn.LayerNorm(48)
        self.fc3 = nn.Linear(48, num_classes)
        self.drop = nn.Dropout(0.05)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = torch.relu(self.ln1(self.fc1(x)))
        x = self.drop(x)
        x = torch.relu(self.ln2(self.fc2(x)))
        return self.fc3(x)


class Profile1DCNN(nn.Module):
    """1D-CNN ablation operating on resampled (24, 3) cluster scan silhouettes."""

    def __init__(self, num_classes: int = 3):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv1d(3, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv1d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(),
        )
        self.head = nn.Sequential(
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, num_classes),
        )

    def forward(self, prof: torch.Tensor) -> torch.Tensor:
        # prof: (B, L, 3) -> (B, 3, L)
        x = prof.permute(0, 2, 1)
        h = self.conv(x)
        avg_p = h.mean(dim=-1)
        max_p = h.max(dim=-1).values
        return self.head(torch.cat([avg_p, max_p], dim=-1))


def _episode_split(ep_ids: np.ndarray, val_frac: float = 0.20, seed: int = 42) -> Tuple[np.ndarray, np.ndarray]:
    unique_eps = np.unique(ep_ids)
    rng = np.random.default_rng(seed)
    rng.shuffle(unique_eps)
    n_val = max(1, int(len(unique_eps) * val_frac))
    val_set = set(unique_eps[:n_val].tolist())
    val_mask = np.array([int(e) in val_set for e in ep_ids], dtype=bool)
    return ~val_mask, val_mask


def _calibrate_temperature(logits: np.ndarray, labels: np.ndarray) -> float:
    """Grid-search scalar temperature minimizing validation NLL."""
    best_t = 1.0
    best_nll = float("inf")
    for t in np.linspace(0.6, 2.5, 39):
        scaled = logits / t
        scaled -= np.max(scaled, axis=1, keepdims=True)
        probs = np.exp(scaled)
        probs /= np.sum(probs, axis=1, keepdims=True)
        nll = -float(np.mean(np.log(np.clip(probs[np.arange(len(labels)), labels], 1e-7, 1.0))))
        if nll < best_nll:
            best_nll = nll
            best_t = float(t)
    return best_t


def train_and_benchmark(dataset_path: str, model_out: str, meta_out: str, epochs: int = 35):
    data = np.load(dataset_path)
    X = data["clf_features"].astype(np.float32)
    P = data["clf_profiles"].astype(np.float32)
    y = data["clf_labels"].astype(np.int64)
    ep_ids = data["clf_ep_ids"] if "clf_ep_ids" in data else np.arange(len(y)) // 100

    train_mask, val_mask = _episode_split(ep_ids, val_frac=0.20, seed=42)
    X_tr, X_va = X[train_mask], X[val_mask]
    P_tr, P_va = P[train_mask], P[val_mask]
    y_tr, y_va = y[train_mask], y[val_mask]

    mean = np.mean(X_tr, axis=0)
    std = np.maximum(np.std(X_tr, axis=0), 1e-4)
    X_tr_n = (X_tr - mean) / std
    X_va_n = (X_va - mean) / std

    # Damped square-root class balancing prevents excess wall->person false positives
    counts = np.bincount(y_tr, minlength=3).astype(np.float32)
    cw_raw = np.sum(counts) / (3.0 * np.maximum(counts, 1.0))
    cw = np.sqrt(cw_raw)

    results: Dict[str, Dict] = {}

    # 1. Random Forest baseline + feature importance
    t0 = time.perf_counter()
    rf = RandomForestClassifier(
        n_estimators=120,
        max_depth=16,
        min_samples_leaf=3,
        class_weight="balanced_subsample",
        n_jobs=-1,
        random_state=42,
    )
    rf.fit(X_tr_n, y_tr)
    rf_pred = rf.predict(X_va_n)
    t_rf_inf0 = time.perf_counter()
    _ = rf.predict_proba(X_va_n[:10])
    rf_ms_10 = (time.perf_counter() - t_rf_inf0) * 1000.0
    rf_f1 = float(f1_score(y_va, rf_pred, average="macro"))
    rf_cm = confusion_matrix(y_va, rf_pred, labels=[0, 1, 2])
    wall_fp_rf = float(rf_cm[0, 2] / max(1, np.sum(rf_cm[0])))
    results["random_forest"] = {
        "macro_f1": round(rf_f1, 4),
        "wall_as_person_fp": round(wall_fp_rf, 4),
        "train_sec": round(time.perf_counter() - t0, 2),
        "infer_10_ms": round(rf_ms_10, 3),
    }

    # Top-5 feature importances
    imp_idx = np.argsort(rf.feature_importances_)[::-1][:6]
    top_feats = [(FEATURE_NAMES[i], round(float(rf.feature_importances_[i]), 4)) for i in imp_idx]
    print(f"[CLF] RandomForest macro-F1={rf_f1:.4f}, wall->person FP={wall_fp_rf:.4f} | Top features: {top_feats}")

    # 2. 1D-CNN spatial ablation (single-frame profile only)
    torch.manual_seed(42)
    cnn = Profile1DCNN(num_classes=3)
    opt_c = torch.optim.AdamW(cnn.parameters(), lr=2e-3, weight_decay=1e-4)
    crit = nn.CrossEntropyLoss(weight=torch.tensor(cw))
    dl_c = DataLoader(TensorDataset(torch.from_numpy(P_tr), torch.from_numpy(y_tr)), batch_size=256, shuffle=True)
    for _ in range(min(epochs, 25)):
        cnn.train()
        for bx, by in dl_c:
            opt_c.zero_grad()
            loss = crit(cnn(bx), by)
            loss.backward()
            opt_c.step()
    cnn.eval()
    with torch.no_grad():
        cnn_logits = cnn(torch.from_numpy(P_va)).numpy()
    cnn_pred = np.argmax(cnn_logits, axis=1)
    cnn_f1 = float(f1_score(y_va, cnn_pred, average="macro"))
    cnn_cm = confusion_matrix(y_va, cnn_pred, labels=[0, 1, 2])
    results["1d_cnn_spatial"] = {
        "macro_f1": round(cnn_f1, 4),
        "wall_as_person_fp": round(float(cnn_cm[0, 2] / max(1, np.sum(cnn_cm[0]))), 4),
    }
    print(f"[CLF] 1D-CNN (spatial-only ablation) macro-F1={cnn_f1:.4f}")

    # 3. Production Compact MLP (20 -> 64 -> 32 -> 3)
    torch.manual_seed(42)
    mlp = CompactMLP(in_dim=X.shape[1], num_classes=3)
    opt = torch.optim.AdamW(mlp.parameters(), lr=2.5e-3, weight_decay=3e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    crit_mlp = nn.CrossEntropyLoss(weight=torch.tensor(cw), label_smoothing=0.03)
    dl = DataLoader(TensorDataset(torch.from_numpy(X_tr_n), torch.from_numpy(y_tr)), batch_size=256, shuffle=True)

    best_f1 = -1.0
    best_state = None
    for ep in range(epochs):
        mlp.train()
        for bx, by in dl:
            # Mild Gaussian jitter augmentation on tabular features
            bx_aug = bx + 0.015 * torch.randn_like(bx)
            opt.zero_grad()
            loss = crit_mlp(mlp(bx_aug), by)
            loss.backward()
            opt.step()
        sched.step()

        mlp.eval()
        with torch.no_grad():
            va_logits = mlp(torch.from_numpy(X_va_n)).numpy()
        f1 = float(f1_score(y_va, np.argmax(va_logits, axis=1), average="macro"))
        if f1 > best_f1:
            best_f1 = f1
            best_state = {k: v.detach().cpu().numpy().copy() for k, v in mlp.state_dict().items()}

    mlp.load_state_dict({k: torch.from_numpy(v) for k, v in best_state.items()})
    mlp.eval()
    with torch.no_grad():
        va_logits = mlp(torch.from_numpy(X_va_n)).numpy()
    temp = _calibrate_temperature(va_logits, y_va)
    mlp_pred = np.argmax(va_logits, axis=1)
    mlp_cm = confusion_matrix(y_va, mlp_pred, labels=[0, 1, 2])
    wall_fp_mlp = float(mlp_cm[0, 2] / max(1, np.sum(mlp_cm[0])))

    results["compact_mlp"] = {
        "macro_f1": round(best_f1, 4),
        "wall_as_person_fp": round(wall_fp_mlp, 4),
        "temperature": round(temp, 3),
        "confusion_matrix": mlp_cm.tolist(),
    }

    print(f"[CLF] CompactMLP macro-F1={best_f1:.4f}, wall->person FP={wall_fp_mlp:.4f}, T={temp:.3f}")
    print(classification_report(y_va, mlp_pred, target_names=list(CLASS_NAMES), digits=4))

    os.makedirs(os.path.dirname(model_out), exist_ok=True)
    np.savez(
        model_out,
        mean=mean.astype(np.float32),
        std=std.astype(np.float32),
        temperature=np.float32(temp),
        fc1_w=best_state["fc1.weight"].astype(np.float32),
        fc1_b=best_state["fc1.bias"].astype(np.float32),
        ln1_w=best_state["ln1.weight"].astype(np.float32),
        ln1_b=best_state["ln1.bias"].astype(np.float32),
        fc2_w=best_state["fc2.weight"].astype(np.float32),
        fc2_b=best_state["fc2.bias"].astype(np.float32),
        ln2_w=best_state["ln2.weight"].astype(np.float32),
        ln2_b=best_state["ln2.bias"].astype(np.float32),
        fc3_w=best_state["fc3.weight"].astype(np.float32),
        fc3_b=best_state["fc3.bias"].astype(np.float32),
    )

    meta = {}
    if os.path.exists(meta_out):
        try:
            with open(meta_out, "r") as f:
                meta = json.load(f)
        except Exception:
            meta = {}
    meta["classifier"] = {
        "feature_names": list(FEATURE_NAMES),
        "class_names": list(CLASS_NAMES),
        "top_rf_features": top_feats,
        "benchmarks": results,
        "num_train": int(len(y_tr)),
        "num_val": int(len(y_va)),
    }
    with open(meta_out, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[CLF] Exported pure-NumPy MLP weights -> {model_out}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default=os.path.expanduser("~/AMR/AMR-main/ml_datasets/synthetic_lidar_dataset.npz"))
    parser.add_argument("--out", default=os.path.expanduser("~/AMR/AMR-main/src/agv_navigation/models/obstacle_classifier.npz"))
    parser.add_argument("--meta", default=os.path.expanduser("~/AMR/AMR-main/src/agv_navigation/models/model_metadata.json"))
    parser.add_argument("--epochs", type=int, default=35)
    args = parser.parse_args()
    train_and_benchmark(args.dataset, args.out, args.meta, args.epochs)


if __name__ == "__main__":
    main()
