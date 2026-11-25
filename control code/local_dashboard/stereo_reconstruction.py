"""Dual-camera stereo 3D centerline reconstruction for OPHIS catheter captures.

Uses the calibrated D405 color + Ace Pro side-view rig (stereo_calibration.json)
to triangulate 2D catheter centerlines via epipolar-constrained polyline matching,
producing 3D centerlines, arc lengths, tangents, bend angles, and reprojection errors
in the D405 camera coordinate frame (mm).
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

import cv2
import numpy as np

from device_plugins import shape3d
from stereo_calibration import _d405_pinhole_points

DEFAULT_MAX_REPROJ_PX = 25.0
DEFAULT_STATIONS = 21


def _skew(vector):
    x, y, z = np.asarray(vector, dtype=np.float64).ravel()
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def arc_lengths(points):
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 2:
        return np.zeros(len(pts), dtype=np.float64)
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def resample_polyline(points, count):
    pts = np.asarray(points, dtype=np.float64)
    s = arc_lengths(pts)
    if len(pts) < 2 or s[-1] <= 1e-9:
        return pts.copy()
    query = np.linspace(0.0, s[-1], count)
    return np.column_stack([np.interp(query, s, pts[:, col]) for col in range(pts.shape[1])])


def deproject_d405_numpy(pixel, depth_mm, color_intrinsics):
    """Pure-NumPy D405 pixel deprojection matching librealsense inverse Brown-Conrady."""
    matrix = np.array(
        [[color_intrinsics['fx'], 0.0, color_intrinsics['ppx']],
         [0.0, color_intrinsics['fy'], color_intrinsics['ppy']],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    coeffs = np.asarray(color_intrinsics.get('coeffs', [0.0] * 5), dtype=np.float64)
    model = str(color_intrinsics.get('model', '')).split('.')[-1]
    model_id = color_intrinsics.get('model_id')
    pt = np.asarray(pixel, dtype=np.float64).reshape(1, 2)
    if np.any(coeffs):
        if model_id in (1, 2) or model in ('modified_brown_conrady', 'inverse_brown_conrady'):
            undist = _d405_pinhole_points(pt, matrix, coeffs)[0]
            x = (float(undist[0]) - color_intrinsics['ppx']) / color_intrinsics['fx']
            y = (float(undist[1]) - color_intrinsics['ppy']) / color_intrinsics['fy']
        else:
            undist = cv2.undistortPoints(pt.reshape(1, 1, 2), matrix, coeffs).reshape(2)
            x, y = float(undist[0]), float(undist[1])
    else:
        x = (float(pt[0, 0]) - color_intrinsics['ppx']) / color_intrinsics['fx']
        y = (float(pt[0, 1]) - color_intrinsics['ppy']) / color_intrinsics['fy']
    z = float(depth_mm)
    return [x * z, y * z, z]


@dataclass(frozen=True)
class StereoRig:
    d405_K: np.ndarray
    d405_coeffs: np.ndarray
    d405_undistort_realsense: bool
    side_K: np.ndarray
    side_dist: np.ndarray
    R_d405_to_side: np.ndarray
    t_d405_to_side_mm: np.ndarray
    P_d405: np.ndarray
    P_side: np.ndarray
    F_d405_to_side: np.ndarray
    baseline_mm: float
    stereo_rms_px: float | None

    @classmethod
    def from_dict(cls, data):
        d405 = data['d405']
        side = data['side_view']
        d405_K = np.asarray(d405['camera_matrix'], dtype=np.float64)
        factory = d405.get('factory_intrinsics', {})
        d405_coeffs = np.asarray(factory.get('coeffs', d405.get('stereo_dist_coeffs', [0.0] * 5)), dtype=np.float64)
        model = str(factory.get('model', '')).split('.')[-1]
        model_id = factory.get('model_id')
        d405_undistort_realsense = (
            d405.get('stereo_points') == 'undistorted_pixels'
            or model_id in (1, 2)
            or model in ('modified_brown_conrady', 'inverse_brown_conrady')
        )
        side_K = np.asarray(side['camera_matrix'], dtype=np.float64)
        side_dist = np.asarray(side['dist_coeffs'], dtype=np.float64).reshape(-1)
        stereo = data.get('stereo') if isinstance(data.get('stereo'), dict) else data
        R = np.asarray(stereo['R_d405_to_side'], dtype=np.float64).reshape(3, 3)
        t = np.asarray(stereo['t_d405_to_side_mm'], dtype=np.float64).reshape(3, 1)
        P_d405 = d405_K @ np.hstack((np.eye(3, dtype=np.float64), np.zeros((3, 1), dtype=np.float64)))
        P_side = side_K @ np.hstack((R, t))
        E = _skew(t) @ R
        F = np.linalg.inv(side_K).T @ E @ np.linalg.inv(d405_K)
        rms = stereo.get('rms_px', data.get('stereo_rms_px'))
        return cls(
            d405_K=d405_K,
            d405_coeffs=d405_coeffs,
            d405_undistort_realsense=d405_undistort_realsense,
            side_K=side_K,
            side_dist=side_dist,
            R_d405_to_side=R,
            t_d405_to_side_mm=t,
            P_d405=P_d405,
            P_side=P_side,
            F_d405_to_side=F,
            baseline_mm=float(stereo.get('baseline_mm', data.get('baseline_mm', np.linalg.norm(t)))),
            stereo_rms_px=float(rms) if rms is not None else None,
        )

    @classmethod
    def from_json(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding='utf-8')))

    def undistort_d405(self, points_px):
        pts = np.asarray(points_px, dtype=np.float64).reshape(-1, 2)
        if not np.any(self.d405_coeffs):
            return pts.copy()
        if self.d405_undistort_realsense:
            return _d405_pinhole_points(pts, self.d405_K, self.d405_coeffs).astype(np.float64)
        return cv2.undistortPoints(
            pts.reshape(-1, 1, 2), self.d405_K, self.d405_coeffs, P=self.d405_K
        ).reshape(-1, 2)

    def undistort_side(self, points_px):
        pts = np.asarray(points_px, dtype=np.float64).reshape(-1, 2)
        if not np.any(self.side_dist):
            return pts.copy()
        return cv2.undistortPoints(
            pts.reshape(-1, 1, 2), self.side_K, self.side_dist, P=self.side_K
        ).reshape(-1, 2)

    def project_undistorted(self, points_3d_mm):
        pts = np.asarray(points_3d_mm, dtype=np.float64).reshape(-1, 3)
        hom = np.hstack((pts, np.ones((len(pts), 1), dtype=np.float64)))
        p1 = (self.P_d405 @ hom.T).T
        p2 = (self.P_side @ hom.T).T
        uv1 = p1[:, :2] / p1[:, 2:3]
        uv2 = p2[:, :2] / p2[:, 2:3]
        return uv1, uv2

    def triangulate(self, d405_undist_px, side_undist_px):
        p1 = np.asarray(d405_undist_px, dtype=np.float64).reshape(-1, 2).T
        p2 = np.asarray(side_undist_px, dtype=np.float64).reshape(-1, 2).T
        hom = cv2.triangulatePoints(self.P_d405, self.P_side, p1, p2)
        pts3d = (hom[:3] / hom[3:4]).T
        z_d405 = pts3d[:, 2]
        pts_side = (self.R_d405_to_side @ pts3d.T + self.t_d405_to_side_mm).T
        z_side = pts_side[:, 2]
        uv1, uv2 = self.project_undistorted(pts3d)
        err1 = np.linalg.norm(uv1 - p1.T, axis=1)
        err2 = np.linalg.norm(uv2 - p2.T, axis=1)
        in_front = (z_d405 > 1.0) & (z_side > 1.0) & np.isfinite(pts3d).all(axis=1)
        return pts3d, err1, err2, in_front


def _match_epipolar_on_polyline(line_abc, poly, poly_u, target_u, min_u):
    """Find the best point on a 2D polyline intersecting or nearest to epipolar line ax+by+c=0."""
    a, b, c = line_abc
    norm = math.hypot(a, b)
    if norm < 1e-12:
        return None
    signed = (poly[:, 0] * a + poly[:, 1] * b + c) / norm
    candidates = []
    for i in range(len(poly) - 1):
        d0, d1 = float(signed[i]), float(signed[i + 1])
        if d0 == 0.0:
            u_cand = float(poly_u[i])
            if u_cand >= min_u - 1e-6:
                candidates.append((0.0, abs(u_cand - target_u), u_cand, poly[i]))
        elif (d0 < 0.0 < d1) or (d1 < 0.0 < d0):
            alpha = -d0 / (d1 - d0)
            u_cand = float(poly_u[i] + alpha * (poly_u[i + 1] - poly_u[i]))
            if u_cand >= min_u - 1e-6:
                pt = poly[i] + alpha * (poly[i + 1] - poly[i])
                candidates.append((0.0, abs(u_cand - target_u), u_cand, pt))
    if candidates:
        candidates.sort(key=lambda item: (item[1], item[0]))
        _, _, u_best, pt_best = candidates[0]
        return u_best, pt_best, 0.0

    # Fallback: closest polyline vertex respecting monotonicity.
    valid_mask = poly_u >= min_u - 1e-6
    if not np.any(valid_mask):
        return None
    idxs = np.flatnonzero(valid_mask)
    dists = np.abs(signed[idxs])
    # Combine epipolar distance with mild arc-length proximity tie-breaking.
    best_local = int(np.argmin(dists + 2.0 * np.abs(poly_u[idxs] - target_u)))
    idx = int(idxs[best_local])
    return float(poly_u[idx]), poly[idx], float(dists[best_local])


def _fit_proximal_tangent(center, distances):
    """Fit a robust 3D proximal tangent at the root using an IRLS quadratic over the proximal window."""
    length = float(distances[-1])
    if length <= 1e-6 or len(center) < 6:
        return None
    window = min(20.0, 0.68 * length)
    mask = distances <= window
    if np.count_nonzero(mask) < 6:
        mask = np.arange(len(center)) < min(6, len(center))
    pts = center[mask]
    s = distances[mask] / max(window, 1e-6)
    design = np.column_stack((np.ones_like(s), s, s * s))
    weights = np.ones(len(s), dtype=np.float64)
    for _ in range(3):
        w = np.sqrt(weights)[:, None]
        terms = np.linalg.lstsq(design * w, pts * w, rcond=None)[0]
        residual = np.linalg.norm(design @ terms - pts, axis=1)
        scale = max(0.15, 1.6 * float(np.median(residual)))
        weights = np.minimum(1.0, scale / np.maximum(residual, 1e-6))
    tangent = terms[1]
    norm = float(np.linalg.norm(tangent))
    if norm < 1e-6:
        return None
    tangent = tangent / norm
    if float(tangent @ (pts[-1] - pts[0])) < 0:
        tangent = -tangent
    return tangent


def bend_azimuth_deg(shaft_dir, tip_tangent, ref_x=None):
    """Compute bend azimuth phi in degrees around shaft_dir; None if nearly straight."""
    z = np.asarray(shaft_dir, dtype=np.float64)
    t = np.asarray(tip_tangent, dtype=np.float64)
    z_norm = np.linalg.norm(z)
    t_norm = np.linalg.norm(t)
    if z_norm < 1e-6 or t_norm < 1e-6:
        return None
    z = z / z_norm
    t = t / t_norm
    if ref_x is None:
        ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    else:
        ref = np.asarray(ref_x, dtype=np.float64)
    x = ref - float(np.dot(ref, z)) * z
    x_norm = np.linalg.norm(x)
    if x_norm < 1e-6:
        return None
    x = x / x_norm
    y = np.cross(z, x)
    tx = float(np.dot(t, x))
    ty = float(np.dot(t, y))
    if math.hypot(tx, ty) < 1e-3:
        return None
    return math.degrees(math.atan2(ty, tx))


def reconstruct_stereo_centerline(
    rig,
    d405_curve_px,
    side_curve_px,
    *,
    length_mm=30.0,
    max_reproj_px=DEFAULT_MAX_REPROJ_PX,
    stations=DEFAULT_STATIONS,
    shaft_direction=None,
):
    """Triangulate a 3D catheter centerline from paired D405 and side-view 2D curves."""
    invalid = dict(
        valid=False,
        reason='双目三维重建不可用',
        angle_deg=None,
        angle_vs_d405_shaft_deg=None,
        bend_azimuth_deg=None,
        centerline_camera_mm=[],
        length_mm=None,
        root_tangent=None,
        tip_tangent=None,
        reproj_rms_px=None,
        d405_reproj_rms_px=None,
        side_reproj_rms_px=None,
        matched_stations=0,
    )
    if not d405_curve_px or not side_curve_px or len(d405_curve_px) < 7 or len(side_curve_px) < 7:
        return dict(invalid, reason='单视角或双视角二维中心线缺失')

    d405_u = rig.undistort_d405(d405_curve_px)
    side_u = rig.undistort_side(side_curve_px)
    d405_s = arc_lengths(d405_u)
    side_s = arc_lengths(side_u)
    if d405_s[-1] < 5.0 or side_s[-1] < 5.0:
        return dict(invalid, reason='二维中心线像素长度过短')

    d405_u_norm = d405_s / d405_s[-1]
    side_dense = resample_polyline(side_u, 101)
    side_dense_s = arc_lengths(side_dense)
    side_dense_u = side_dense_s / side_dense_s[-1]

    matched_d405 = []
    matched_side = []
    matched_indices = []
    min_u = 0.0
    n_pts = len(d405_u)

    for idx in range(n_pts):
        pt_d = d405_u[idx]
        if idx == 0:
            pt_s = side_u[0]
            min_u = 0.0
        elif idx == n_pts - 1:
            pt_s = side_u[-1]
        else:
            line = rig.F_d405_to_side @ np.array([pt_d[0], pt_d[1], 1.0], dtype=np.float64)
            match = _match_epipolar_on_polyline(line, side_dense, side_dense_u, float(d405_u_norm[idx]), min_u)
            if match is None:
                continue
            u_best, pt_s, _ = match
            min_u = max(min_u, u_best)
        matched_d405.append(pt_d)
        matched_side.append(pt_s)
        matched_indices.append(idx)

    if len(matched_d405) < 7:
        return dict(invalid, reason='极线匹配点数不足')

    pts3d, err_d405, err_side, in_front = rig.triangulate(matched_d405, matched_side)
    reproj_pair = np.sqrt(0.5 * (err_d405 ** 2 + err_side ** 2))
    keep = in_front & (err_d405 <= max_reproj_px) & (err_side <= max_reproj_px)
    kept_indices = np.flatnonzero(keep)
    if len(kept_indices) < 7 or kept_indices[0] > 3 or kept_indices[-1] < len(matched_d405) - 4:
        return dict(
            invalid,
            reason='双目三角化重投影误差过大或端点不满足极线约束',
            reproj_rms_px=float(np.sqrt(np.mean(reproj_pair ** 2))) if len(reproj_pair) else None,
            d405_reproj_rms_px=float(np.sqrt(np.mean(err_d405 ** 2))) if len(err_d405) else None,
            side_reproj_rms_px=float(np.sqrt(np.mean(err_side ** 2))) if len(err_side) else None,
            matched_stations=int(len(kept_indices)),
        )

    good_pts = pts3d[keep]
    # Light 3-point smoothing on interior stations before equal-arc-length resampling.
    smoothed = good_pts.copy()
    if len(smoothed) >= 3:
        smoothed[1:-1] = (smoothed[:-2] + 2.0 * smoothed[1:-1] + smoothed[2:]) / 4.0

    center = resample_polyline(smoothed, stations)
    distances = arc_lengths(center)
    total_length = float(distances[-1])
    if not (0.5 * length_mm <= total_length <= 1.7 * length_mm):
        return dict(
            invalid,
            reason=f'双目重建弧长异常（{total_length:.1f} mm）',
            length_mm=total_length,
            reproj_rms_px=float(np.sqrt(np.mean(reproj_pair[keep] ** 2))),
            d405_reproj_rms_px=float(np.sqrt(np.mean(err_d405[keep] ** 2))),
            side_reproj_rms_px=float(np.sqrt(np.mean(err_side[keep] ** 2))),
            matched_stations=int(np.count_nonzero(keep)),
        )

    root_tangent = _fit_proximal_tangent(center, distances)
    tip_tangent = shape3d.fit_distal_tangent(center, distances, np.arange(len(center)))
    if root_tangent is None or tip_tangent is None:
        return dict(
            invalid,
            reason='双目中心线根部或末端切线拟合失败',
            centerline_camera_mm=center.tolist(),
            length_mm=total_length,
            reproj_rms_px=float(np.sqrt(np.mean(reproj_pair[keep] ** 2))),
            d405_reproj_rms_px=float(np.sqrt(np.mean(err_d405[keep] ** 2))),
            side_reproj_rms_px=float(np.sqrt(np.mean(err_side[keep] ** 2))),
            matched_stations=int(np.count_nonzero(keep)),
        )

    angle_deg = shape3d.bend_angle(root_tangent, tip_tangent)
    angle_vs_shaft = None
    if shaft_direction is not None:
        shaft_arr = np.asarray(shaft_direction, dtype=np.float64)
        if shaft_arr.shape == (3,) and np.isfinite(shaft_arr).all() and np.linalg.norm(shaft_arr) > 1e-6:
            angle_vs_shaft = shape3d.bend_angle(shaft_arr / np.linalg.norm(shaft_arr), tip_tangent)

    return dict(
        valid=True,
        reason='双目三维中心线重建有效',
        angle_deg=float(angle_deg),
        angle_vs_d405_shaft_deg=float(angle_vs_shaft) if angle_vs_shaft is not None else None,
        bend_azimuth_deg=bend_azimuth_deg(root_tangent, tip_tangent),
        centerline_camera_mm=center.tolist(),
        length_mm=total_length,
        root_tangent=root_tangent.tolist(),
        tip_tangent=tip_tangent.tolist(),
        reproj_rms_px=float(np.sqrt(np.mean(reproj_pair[keep] ** 2))),
        d405_reproj_rms_px=float(np.sqrt(np.mean(err_d405[keep] ** 2))),
        side_reproj_rms_px=float(np.sqrt(np.mean(err_side[keep] ** 2))),
        matched_stations=int(np.count_nonzero(keep)),
    )
