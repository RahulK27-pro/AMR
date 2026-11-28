"""Pure-NumPy runtime inference for the 3-class LiDAR obstacle classifier (WALL / CLUTTER / PERSON)."""

import math
import os
from typing import Optional

import numpy as np

from .lidar_clustering import CLASS_CLUTTER, CLASS_PERSON, CLASS_WALL


def _layer_norm(x: np.ndarray, w: np.ndarray, b: np.ndarray, eps: float = 1e-5) -> np.ndarray:
    mean = np.mean(x, axis=-1, keepdims=True)
    var = np.var(x, axis=-1, keepdims=True)
    return ((x - mean) / np.sqrt(var + eps)) * w + b


def _softmax(logits: np.ndarray) -> np.ndarray:
    shifted = logits - np.max(logits, axis=-1, keepdims=True)
    ex = np.exp(shifted)
    return ex / np.sum(ex, axis=-1, keepdims=True)


class ObstacleClassifierRuntime:
    """Loads exported MLP or Tree-Ensemble weights (.npz) and evaluates probabilities in pure NumPy.

    Supports both calibrated CompactMLP ('mlp') and scikit-learn RandomForest/ExtraTrees
    ('tree_ensemble') exported to pure-NumPy arrays, with an automatic heuristic fallback.
    """

    def __init__(self, model_path: Optional[str] = None):
        self.loaded = False
        self.model_type = "heuristic"
        self.weights = {}
        if model_path and os.path.exists(model_path):
            self.load(model_path)

    def load(self, model_path: str):
        data = np.load(model_path)
        if "model_type" in data:
            mt = data["model_type"]
            self.model_type = str(mt.item() if mt.ndim == 0 else mt[0])
        else:
            self.model_type = "tree_ensemble" if "tree_features" in data else "mlp"

        if self.model_type == "tree_ensemble":
            n_trees = int(data["n_trees"])
            max_nodes = int(data["max_nodes"])
            offsets = np.arange(n_trees, dtype=np.int32) * max_nodes
            t_left = data["tree_left"].astype(np.int32)
            t_right = data["tree_right"].astype(np.int32)
            self.weights = {
                "mean": data["mean"].astype(np.float32),
                "std": data["std"].astype(np.float32),
                "flat_feat": data["tree_features"].astype(np.int32).ravel(),
                "flat_thresh": data["tree_thresholds"].astype(np.float32).ravel(),
                "flat_left": (t_left + offsets[:, None]).ravel(),
                "flat_right": (t_right + offsets[:, None]).ravel(),
                "flat_values": data["tree_values"].astype(np.float32).reshape(n_trees * max_nodes, 3),
                "offsets": offsets,
                "n_trees": n_trees,
            }
        else:
            self.weights = {k: data[k].astype(np.float32) for k in data.files if k != "model_type"}
        self.loaded = True

    def _predict_tree_ensemble(self, x_norm: np.ndarray) -> np.ndarray:
        """Vectorized 1D pure-NumPy inference over packed decision-tree arrays."""
        w = self.weights
        n_samples = len(x_norm)
        n_trees = w["n_trees"]
        feat = w["flat_feat"]
        thresh = w["flat_thresh"]
        left = w["flat_left"]
        right = w["flat_right"]
        values = w["flat_values"]

        nodes = np.broadcast_to(w["offsets"][None, :], (n_samples, n_trees)).copy()
        x_flat = np.ascontiguousarray(x_norm, dtype=np.float32).ravel()
        row_off = np.arange(n_samples, dtype=np.int32)[:, None] * x_norm.shape[1]
        for _ in range(20):
            cur_f = feat[nodes]
            is_leaf = cur_f < 0
            if is_leaf.all():
                break
            vals = x_flat[row_off + np.maximum(0, cur_f)]
            nxt = np.where(vals <= thresh[nodes], left[nodes], right[nodes])
            nodes = np.where(is_leaf, nodes, nxt)

        probs = values[nodes].mean(axis=1)
        row_sums = np.maximum(1e-6, np.sum(probs, axis=-1, keepdims=True))
        return (probs / row_sums).astype(np.float32)

    def predict_proba_batch(self, features: np.ndarray) -> np.ndarray:
        """Return (N, 3) calibrated probabilities for [WALL, CLUTTER, PERSON]."""
        feats = np.asarray(features, dtype=np.float32)
        if feats.ndim == 1:
            feats = feats[None, :]
        if len(feats) == 0:
            return np.empty((0, 3), dtype=np.float32)

        if not self.loaded:
            return self._heuristic_fallback(feats)

        w = self.weights
        x = (feats - w["mean"]) / w["std"]
        if self.model_type == "tree_ensemble":
            return self._predict_tree_ensemble(x)

        h1 = np.maximum(0.0, _layer_norm(x @ w["fc1_w"].T + w["fc1_b"], w["ln1_w"], w["ln1_b"]))
        h2 = np.maximum(0.0, _layer_norm(h1 @ w["fc2_w"].T + w["fc2_b"], w["ln2_w"], w["ln2_b"]))
        logits = (h2 @ w["fc3_w"].T + w["fc3_b"]) / max(0.25, float(w["temperature"]))
        return _softmax(logits).astype(np.float32)

    @staticmethod
    def bayesian_filter(prev_probs: np.ndarray, obs_probs: np.ndarray, alpha: float = 0.35) -> np.ndarray:
        """Smooth class probabilities over time without freezing genuine state transitions."""
        prior = np.clip(prev_probs, 1e-4, 1.0)
        like = np.clip(obs_probs, 1e-4, 1.0)
        post = (prior ** (1.0 - alpha)) * (like ** alpha)
        return (post / np.sum(post)).astype(np.float32)

    @staticmethod
    def _heuristic_fallback(feats: np.ndarray) -> np.ndarray:
        """Conservative geometry + velocity fallback when .npz weights are unavailable."""
        out = np.zeros((len(feats), 3), dtype=np.float32)
        for i, f in enumerate(feats):
            width = float(f[1])
            linearity = float(f[7])
            wall_edt = float(f[13])
            speed = float(f[15])
            net_ratio = float(f[17])
            age = math.expm1(float(f[19]))

            if (wall_edt < 0.14 and width > 0.65) or (width > 1.15 and linearity > 0.94):
                out[i] = [0.90, 0.08, 0.02]
            elif speed >= 0.22 and net_ratio >= 0.45 and age >= 3.0 and width <= 0.85:
                out[i] = [0.05, 0.15, 0.80]
            else:
                out[i] = [0.10, 0.80, 0.10]
        return out
