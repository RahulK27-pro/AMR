"""2D LiDAR polar breakpoint segmentation, Euclidean clustering, and 20-D feature extraction."""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

CLASS_WALL = 0
CLASS_CLUTTER = 1
CLASS_PERSON = 2
CLASS_NAMES = ("WALL", "CLUTTER", "PERSON")

FEATURE_NAMES = (
    "log_num_points",
    "chord_width",
    "max_span",
    "point_density",
    "expected_density_ratio",
    "pca_std_major",
    "pca_std_minor",
    "linearity",
    "chord_bulge",
    "circle_fit_radius",
    "circle_fit_rms",
    "jump_left_m",
    "jump_right_m",
    "wall_edt_mean_m",
    "wall_edt_min_m",
    "kf_speed",
    "velocity_variance",
    "net_displacement_ratio",
    "width_std",
    "log_track_age",
)


@dataclass
class ClusterCandidate:
    """Segmented 2D LiDAR cluster with local/map coordinates and instantaneous features."""

    points_local: np.ndarray          # (N, 2) in base_link / laser frame
    points_map: np.ndarray            # (N, 2) in map frame
    centroid_local: np.ndarray        # (2,)
    centroid_map: np.ndarray          # (2,)
    num_points: int
    chord_width: float
    radius_est: float                 # Estimated physical obstacle radius (m)
    wall_edt_mean: float
    wall_edt_min: float
    geom_features: np.ndarray         # First 15 instantaneous geometric/map features
    is_leg_pair: bool = False


class MapContext:
    """Precomputed static occupancy distance transform and spatial gradients for O(1) lookups."""

    def __init__(
        self,
        occ_grid: Optional[np.ndarray] = None,
        resolution: float = 0.05,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ):
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.edt_grid: Optional[np.ndarray] = None
        self.grad_x: Optional[np.ndarray] = None
        self.grad_y: Optional[np.ndarray] = None
        self.height = 0
        self.width = 0
        if occ_grid is not None:
            self.update(occ_grid, resolution, origin_x, origin_y)

    def update(self, occ_grid: np.ndarray, resolution: float, origin_x: float, origin_y: float):
        """Recompute Euclidean distance transform (m) and unit repulsion gradients from binary occ grid."""
        occ = np.asarray(occ_grid, dtype=bool)
        if occ.ndim != 2 or occ.size == 0:
            self.edt_grid = None
            return
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = occ.shape

        # Distance in meters from every free cell to nearest occupied wall cell
        edt = distance_transform_edt(~occ).astype(np.float32) * np.float32(self.resolution)
        gy, gx = np.gradient(edt)
        norm = np.hypot(gx, gy) + 1e-6
        self.edt_grid = edt
        self.grad_x = (gx / norm).astype(np.float32)
        self.grad_y = (gy / norm).astype(np.float32)

    def query_edt(self, pts_map: np.ndarray) -> np.ndarray:
        """Query static-wall distance (m) for (N, 2) map points."""
        if self.edt_grid is None or len(pts_map) == 0:
            return np.full(len(pts_map), 3.0, dtype=np.float32)
        gx = np.floor((pts_map[:, 0] - self.origin_x) / self.resolution).astype(np.int32)
        gy = np.floor((pts_map[:, 1] - self.origin_y) / self.resolution).astype(np.int32)
        inside = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        out = np.zeros(len(pts_map), dtype=np.float32)
        out[inside] = self.edt_grid[gy[inside], gx[inside]]
        return np.clip(out, 0.0, 3.0)

    def query_edt_and_grad(self, x: float, y: float) -> Tuple[float, float, float]:
        """Query (edt_m, grad_x, grad_y) at a single map coordinate."""
        if self.edt_grid is None:
            return 3.0, 0.0, 0.0
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return (
                float(min(3.0, self.edt_grid[gy, gx])),
                float(self.grad_x[gy, gx]),
                float(self.grad_y[gy, gx]),
            )
        return 0.0, 0.0, 0.0


def _fit_circle_kasa(pts: np.ndarray) -> Tuple[float, float]:
    """Algebraic Kasa circle fit returning (radius_m, radial_rms_m)."""
    n = len(pts)
    if n < 3:
        return 0.15, 0.02
    x = pts[:, 0]
    y = pts[:, 1]
    u = x - float(x.mean())
    v = y - float(y.mean())
    u2 = u * u
    v2 = v * v
    Suu = float(u2.sum())
    Svv = float(v2.sum())
    Suv = float(u @ v)
    det = Suu * Svv - Suv * Suv
    if abs(det) < 1e-9:
        return 2.0, 0.01
    Suuu = float(u2 @ u)
    Svvv = float(v2 @ v)
    Suvv = float(v2 @ u)
    Svuu = float(u2 @ v)
    b1 = 0.5 * (Suuu + Suvv)
    b2 = 0.5 * (Svvv + Svuu)
    uc = (b1 * Svv - b2 * Suv) / det
    vc = (Suu * b2 - Suv * b1) / det
    r = math.hypot(uc, vc)
    if not math.isfinite(r) or r > 2.0:
        return 2.0, 0.01
    radii = np.hypot(u - uc, v - vc)
    rms = float(np.sqrt(np.mean((radii - r) ** 2)))
    return float(max(0.02, min(2.0, r))), float(min(0.5, rms))


def _compute_cluster_geometry(
    pts_local: np.ndarray,
    pts_map: np.ndarray,
    jump_left: float,
    jump_right: float,
    angle_inc: float,
    map_ctx: Optional[MapContext],
) -> ClusterCandidate:
    """Compute the 15 instantaneous geometric + occlusion + map features for one cluster."""
    n = len(pts_local)
    centroid_loc = pts_local.mean(axis=0)
    centroid_map = pts_map.mean(axis=0)

    cvx = float(pts_local[-1, 0] - pts_local[0, 0])
    cvy = float(pts_local[-1, 1] - pts_local[0, 1])
    chord_vec = np.array([cvx, cvy], dtype=np.float32)
    chord_width = math.hypot(cvx, cvy)
    mean_range = max(0.15, math.hypot(float(centroid_loc[0]), float(centroid_loc[1])))

    # Exact closed-form 2x2 symmetric PCA eigenvalues (40x faster than LAPACK np.linalg.eigh)
    centered = pts_local - centroid_loc
    denom = float(max(1, n - 1))
    ux = centered[:, 0]
    uy = centered[:, 1]
    c_xx = float(np.dot(ux, ux)) / denom
    c_yy = float(np.dot(uy, uy)) / denom
    c_xy = float(np.dot(ux, uy)) / denom
    half_tr = 0.5 * (c_xx + c_yy)
    half_diff = 0.5 * (c_xx - c_yy)
    rad = math.hypot(half_diff, c_xy)
    lam_major = max(0.0, half_tr + rad)
    lam_minor = max(0.0, half_tr - rad)
    std_minor = math.sqrt(lam_minor)
    std_major = math.sqrt(lam_major)
    linearity = (lam_major - lam_minor) / (lam_major + 1e-6)

    if rad > 1e-12:
        if half_diff >= 0.0:
            mx = half_diff + rad
            my = c_xy
        else:
            mx = c_xy
            my = rad - half_diff
        mn = math.hypot(mx, my)
        maj_x, maj_y = mx / mn, my / mn
    else:
        maj_x, maj_y = 1.0, 0.0
    proj_major = ux * maj_x + uy * maj_y
    max_span = float(np.max(proj_major) - np.min(proj_major)) if n > 1 else chord_width

    # Signed chord bulge toward the sensor (positive = convex arc facing LiDAR)
    if chord_width > 1e-4 and n >= 3:
        chord_dir = chord_vec / chord_width
        mid = 0.5 * (pts_local[0] + pts_local[-1])
        nx, ny = -float(chord_dir[1]), float(chord_dir[0])
        if (nx * (-float(mid[0])) + ny * (-float(mid[1]))) < 0.0:
            nx, ny = -nx, -ny
        signed_offsets = (pts_local[:, 0] - pts_local[0, 0]) * nx + (pts_local[:, 1] - pts_local[0, 1]) * ny
        signed_offsets.sort()
        chord_bulge = float(np.clip(signed_offsets[n // 2], -0.5, 0.5))
    else:
        chord_bulge = 0.0

    # Expected vs observed beam density on a solid target at mean_range
    eff_width = max(chord_width, 0.05)
    point_density = float(min(300.0, n / eff_width))
    angular_span = 2.0 * math.atan2(eff_width * 0.5, mean_range)
    expected_pts = max(1.0, angular_span / max(1e-4, angle_inc))
    expected_density_ratio = float(min(3.0, n / expected_pts))

    circle_r, circle_rms = _fit_circle_kasa(pts_local)

    if map_ctx is not None and map_ctx.edt_grid is not None:
        edt_vals = map_ctx.query_edt(pts_map)
        wall_edt_mean = float(np.mean(edt_vals))
        wall_edt_min = float(np.min(edt_vals))
    else:
        wall_edt_mean = 3.0
        wall_edt_min = 3.0

    # Estimate physical collision radius from observed span (avoid slow np.percentile)
    if n >= 3:
        obs_radii = np.hypot(ux, uy)
        k_idx = max(0, min(n - 1, int(0.9 * (n - 1))))
        r_est = float(np.partition(obs_radii, k_idx)[k_idx])
    else:
        r_est = 0.20
    r_est = float(max(0.14, min(0.45, max(r_est, 0.5 * chord_width))))

    geom_features = np.array(
        [
            math.log1p(n),
            chord_width,
            max_span,
            point_density,
            expected_density_ratio,
            std_major,
            std_minor,
            linearity,
            chord_bulge,
            circle_r,
            circle_rms,
            float(np.clip(jump_left, -2.0, 2.0)),
            float(np.clip(jump_right, -2.0, 2.0)),
            wall_edt_mean,
            wall_edt_min,
        ],
        dtype=np.float32,
    )

    return ClusterCandidate(
        points_local=pts_local,
        points_map=pts_map,
        centroid_local=centroid_loc.astype(np.float32),
        centroid_map=centroid_map.astype(np.float32),
        num_points=n,
        chord_width=chord_width,
        radius_est=r_est,
        wall_edt_mean=wall_edt_mean,
        wall_edt_min=wall_edt_min,
        geom_features=geom_features,
        is_leg_pair=False,
    )


def _merge_leg_pairs(
    clusters: List[ClusterCandidate],
    angle_inc: float,
    map_ctx: Optional[MapContext],
) -> List[ClusterCandidate]:
    """Group nearby small leg-sized foreground clusters into a unified pedestrian candidate."""
    if len(clusters) < 2:
        return clusters

    # Eligible single-leg candidates: compact width, foreground (not deep inside static walls)
    eligible = [
        i for i, c in enumerate(clusters)
        if c.chord_width <= 0.25 and c.num_points <= 22 and c.wall_edt_mean >= 0.14
    ]
    used = set()
    merged: List[ClusterCandidate] = []

    for idx_pos, i in enumerate(eligible):
        if i in used:
            continue
        ci = clusters[i]
        best_j = None
        best_d = 0.52  # Max natural bipedal stride separation (m)
        for j in eligible[idx_pos + 1:]:
            if j in used:
                continue
            cj = clusters[j]
            d = float(np.linalg.norm(ci.centroid_local - cj.centroid_local))
            dr_robot = abs(float(np.linalg.norm(ci.centroid_local)) - float(np.linalg.norm(cj.centroid_local)))
            if 0.10 <= d < best_d and dr_robot <= 0.35:
                best_d = d
                best_j = j

        if best_j is not None:
            cj = clusters[best_j]
            pts_loc = np.vstack((ci.points_local, cj.points_local))
            pts_map = np.vstack((ci.points_map, cj.points_map))
            if len(pts_loc) >= 4:
                cand = _compute_cluster_geometry(
                    pts_loc,
                    pts_map,
                    jump_left=float(ci.geom_features[11]),
                    jump_right=float(cj.geom_features[12]),
                    angle_inc=angle_inc,
                    map_ctx=map_ctx,
                )
                cand.is_leg_pair = True
                cand.radius_est = float(max(0.24, min(0.38, cand.radius_est + 0.04)))
                merged.append(cand)
                used.add(i)
                used.add(best_j)

    for idx, c in enumerate(clusters):
        if idx not in used and c.num_points >= 3:
            merged.append(c)
    return merged


def extract_lidar_clusters(
    ranges: np.ndarray,
    angles: np.ndarray,
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    range_min: float = 0.15,
    range_max: float = 8.0,
    angle_inc: Optional[float] = None,
    max_cluster_range: float = 5.5,
    map_ctx: Optional[MapContext] = None,
    min_points: int = 3,
    enable_leg_pair_merge: bool = True,
) -> Tuple[List[ClusterCandidate], np.ndarray, np.ndarray]:
    """Run full-resolution self-masking + Polar Adaptive Breakpoint + Euclidean clustering.

    Returns:
        (clusters, all_external_local_pts, all_external_map_pts)
    """
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = np.asarray(angles, dtype=np.float32)

    valid = (ranges > max(0.08, range_min)) & (ranges < min(12.0, range_max)) & np.isfinite(ranges)
    if not np.any(valid):
        empty = np.empty((0, 2), dtype=np.float32)
        return [], empty, empty

    r = ranges[valid]
    th = angles[valid]
    ox_loc = r * np.cos(th)
    oy_loc = r * np.sin(th)
    r_loc = np.hypot(ox_loc, oy_loc)

    # Exact robot footprint self-masking (matching route_runner.py chassis + wheels)
    is_chassis = r_loc < 0.160
    is_wheel = (np.abs(oy_loc) >= 0.110) & (np.abs(oy_loc) <= 0.165) & (np.abs(ox_loc) <= 0.080)
    ext = (~is_chassis) & (~is_wheel)
    if not np.any(ext):
        empty = np.empty((0, 2), dtype=np.float32)
        return [], empty, empty

    r = r[ext]
    th = th[ext]
    ox_loc = ox_loc[ext]
    oy_loc = oy_loc[ext]

    # Only sort if scan angles aren't already strictly ascending
    if len(th) > 1 and np.any(th[1:] < th[:-1]):
        order = np.argsort(th)
        r = r[order]
        th = th[order]
        ox_loc = ox_loc[order]
        oy_loc = oy_loc[order]

    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    ox_map = pose_x + ox_loc * cos_yaw - oy_loc * sin_yaw
    oy_map = pose_y + ox_loc * sin_yaw + oy_loc * cos_yaw

    all_local = np.column_stack((ox_loc, oy_loc)).astype(np.float32)
    all_map = np.column_stack((ox_map, oy_map)).astype(np.float32)

    # Filter to clustering range while preserving ordered polar rays for occlusion jump cues
    near_idx = np.where(r <= max_cluster_range)[0]
    if len(near_idx) < 2:
        return [], all_local, all_map

    if angle_inc is None:
        diffs = np.diff(th)
        pos_diffs = diffs[diffs > 1e-4]
        angle_inc = float(np.median(pos_diffs)) if len(pos_diffs) > 0 else (2.0 * math.pi / 360.0)

    # Polar Adaptive Breakpoint Detector (ABD) on full-resolution scan (vectorized)
    sin_abd = math.sin(math.radians(10.0))
    idx_a = near_idx[:-1]
    idx_b = near_idx[1:]
    d_theta = th[idx_b] - th[idx_a]
    r_min_pair = np.minimum(r[idx_a], r[idx_b])
    d_euclid = np.hypot(ox_loc[idx_b] - ox_loc[idx_a], oy_loc[idx_b] - oy_loc[idx_a])
    d_thresh = np.clip(
        r_min_pair * (np.sin(np.minimum(d_theta, 0.14)) / max(1e-4, sin_abd)) + 0.08,
        0.12,
        0.32,
    )
    is_break = (d_theta > max(4.5 * angle_inc, 0.09)) | (d_euclid > d_thresh)
    break_pos = np.flatnonzero(is_break) + 1
    segments: List[np.ndarray] = np.split(near_idx, break_pos)

    # Handle -pi / +pi wrap-around seamlessly
    if len(segments) >= 2:
        first_i = int(segments[0][0])
        last_i = int(segments[-1][-1])
        d_wrap = (2.0 * math.pi) - float(th[last_i] - th[first_i])
        d_euclid_wrap = math.hypot(float(ox_loc[first_i] - ox_loc[last_i]), float(oy_loc[first_i] - oy_loc[last_i]))
        r_min_wrap = float(min(r[first_i], r[last_i]))
        d_thresh_wrap = max(0.12, min(0.32, r_min_wrap * (math.sin(min(max(d_wrap, 0.0), 0.14)) / sin_abd) + 0.08))
        if d_wrap <= max(4.5 * angle_inc, 0.09) and d_euclid_wrap <= d_thresh_wrap:
            segments[0] = np.concatenate((segments[-1], segments[0]))
            segments.pop()

    # Heal tiny 1-beam dropout splits inside the same rigid surface (gap <= 0.10 m)
    merged_segments: List[np.ndarray] = []
    for seg in segments:
        if not merged_segments:
            merged_segments.append(seg)
            continue
        prev_end = int(merged_segments[-1][-1])
        curr_start = int(seg[0])
        gap = math.hypot(float(ox_loc[curr_start] - ox_loc[prev_end]), float(oy_loc[curr_start] - oy_loc[prev_end]))
        if gap <= 0.10:
            merged_segments[-1] = np.concatenate((merged_segments[-1], seg))
        else:
            merged_segments.append(seg)

    # Build ClusterCandidates (allowing 2-point sub-clusters briefly for leg-pair assembly)
    raw_candidates: List[ClusterCandidate] = []
    n_total = len(r)
    for seg_arr in merged_segments:
        if len(seg_arr) < (2 if enable_leg_pair_merge else min_points):
            continue
        first_idx = int(seg_arr[0])
        last_idx = int(seg_arr[-1])
        prev_beam = (first_idx - 1) % n_total
        next_beam = (last_idx + 1) % n_total
        jump_left = float(r[prev_beam] - r[first_idx])
        jump_right = float(r[next_beam] - r[last_idx])

        cand = _compute_cluster_geometry(
            all_local[seg_arr],
            all_map[seg_arr],
            jump_left=jump_left,
            jump_right=jump_right,
            angle_inc=angle_inc,
            map_ctx=map_ctx,
        )
        raw_candidates.append(cand)

    if enable_leg_pair_merge:
        candidates = _merge_leg_pairs(raw_candidates, angle_inc=angle_inc, map_ctx=map_ctx)
    else:
        candidates = [c for c in raw_candidates if c.num_points >= min_points]

    return candidates, all_local, all_map
