"""Pure-NumPy 3D forward and inverse kinematics modeling for the OPHIS soft catheter.

Operates on stereo-reconstructed 21-point 3D centerlines transformed into each
round's proximal reference frame B_ref (origin at reference root, +Z along the
reference root-to-tip axis, +X from orthogonalized camera X, +Y = Z x X).

Provides:
  - Per-round reference frame alignment and canonical sample extraction
  - Multiple pure-NumPy forward models (Linear, Quadratic, Deadzone-Poly, RBF+Poly)
    predicting 3D unit tip tangent, 3D tip position (mm), and 21-point 3D centerline
  - Grouped leave-one-point-out cross-validation (no leakage across repeats)
  - Constrained multi-start projected Levenberg-Marquardt inverse solver
  - JSON serialization for real-time deployment inside local_dashboard
"""

from collections import defaultdict
import json
import math
from pathlib import Path

import numpy as np

from device_plugins import shape3d
from stereo_reconstruction import arc_lengths


def build_reference_frame(ref_curve_mm):
    """Build right-handed reference frame B_ref from a 21-point reference 3D centerline.

    Returns (origin_mm, R_cam_to_ref) such that p_ref = (p_cam - origin_mm) @ R_cam_to_ref.T.
    """
    curve = np.asarray(ref_curve_mm, dtype=np.float64)
    origin = curve[0].copy()
    z = curve[-1] - curve[0]
    z_norm = float(np.linalg.norm(z))
    if z_norm < 1e-6:
        raise ValueError('参考中心线长度为零')
    z = z / z_norm
    cam_x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(np.dot(cam_x, z))) > 0.9:
        cam_x = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x = cam_x - float(np.dot(cam_x, z)) * z
    x = x / float(np.linalg.norm(x))
    y = np.cross(z, x)
    R = np.vstack((x, y, z))  # rows are basis vectors of B_ref in camera coords
    return origin, R


def _distal_tangent_from_curve(curve_mm):
    pts = np.asarray(curve_mm, dtype=np.float64)
    dists = arc_lengths(pts)
    tan = shape3d.fit_distal_tangent(pts, dists, np.arange(len(pts)))
    if tan is None:
        v = pts[-1] - pts[-4]
        norm = float(np.linalg.norm(v))
        if norm < 1e-6:
            return None
        tan = v / norm
    return tan / float(np.linalg.norm(tan))


def extract_canonical_sample(steady_shape, round_row, *, modality='stereo'):
    """Transform a round's steady 3D centerline into B_ref and extract canonical targets."""
    ref_key = f'{modality}_reference_camera_mm'
    hold_key = f'{modality}_hold_camera_mm'
    ref_cam = steady_shape.get(ref_key)
    hold_cam = steady_shape.get(hold_key)
    if not ref_cam or not hold_cam or len(ref_cam) != 21 or len(hold_cam) != 21:
        return None

    origin, R = build_reference_frame(ref_cam)
    ref_local = (np.asarray(ref_cam, dtype=np.float64) - origin) @ R.T
    hold_local = (np.asarray(hold_cam, dtype=np.float64) - origin) @ R.T
    # Align hold root to origin so fixture translation noise does not pollute centerline shape.
    hold_local = hold_local - hold_local[0]
    ref_local = ref_local - ref_local[0]

    ref_tan = _distal_tangent_from_curve(ref_local)
    hold_tan = _distal_tangent_from_curve(hold_local)
    if ref_tan is None or hold_tan is None:
        return None

    # Rotate B_ref slightly so ref_tan aligns exactly with [0, 0, 1] for clean zero-state tangents.
    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    cross = np.cross(ref_tan, z_axis)
    sin_a = float(np.linalg.norm(cross))
    cos_a = float(np.clip(np.dot(ref_tan, z_axis), -1.0, 1.0))
    if sin_a > 1e-8:
        k = cross / sin_a
        K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]], dtype=np.float64)
        R_align = np.eye(3) + sin_a * K + (1.0 - cos_a) * (K @ K)
        ref_local = ref_local @ R_align.T
        hold_local = hold_local @ R_align.T
        hold_tan = R_align @ hold_tan
        hold_tan = hold_tan / float(np.linalg.norm(hold_tan))

    q_actual = [
        float(round_row[f'actual_hold_q{i}_mm'])
        if round_row.get(f'actual_hold_q{i}_mm') not in (None, '')
        else float(steady_shape['target_mm'][i - 1])
        for i in (1, 2, 3)
    ]
    bend_deg = float(shape3d.bend_angle(z_axis, hold_tan))
    azimuth_deg = (
        math.degrees(math.atan2(float(hold_tan[1]), float(hold_tan[0])))
        if math.hypot(float(hold_tan[0]), float(hold_tan[1])) > 1e-3
        else None
    )

    return dict(
        dataset_group=steady_shape['dataset_group'],
        batch_name=steady_shape['batch_name'],
        quality_flag=steady_shape['quality_flag'],
        ids=list(steady_shape['ids']),
        target_mm=list(steady_shape['target_mm']),
        actual_mm=q_actual,
        repeat=int(steady_shape['repeat']),
        round_slug=steady_shape['round_slug'],
        ref_centerline_mm=ref_local,
        centerline_mm=hold_local,
        tip_pos_mm=hold_local[-1],
        ref_tip_pos_mm=ref_local[-1],
        tip_tangent=hold_tan,
        bend_angle_deg=bend_deg,
        bend_azimuth_deg=azimuth_deg,
        reproj_rms_px=(
            float(round_row['stereo_reproj_rms_px'])
            if round_row.get('stereo_reproj_rms_px') not in (None, '')
            else None
        ),
    )


def _softplus(x, scale=0.6):
    z = np.asarray(x, dtype=np.float64) / scale
    return scale * np.where(z > 20.0, z, np.log1p(np.exp(np.clip(z, -40.0, 20.0))))


def _poly_features(X, degree=2, deadzone_mm=None):
    X = np.atleast_2d(np.asarray(X, dtype=np.float64))
    n = len(X)
    q1, q2, q3 = X[:, 0], X[:, 1], X[:, 2]
    cols = [np.ones(n), q1, q2, q3]
    if deadzone_mm is not None:
        d1, d2, d3 = deadzone_mm
        a1 = _softplus(q1 - d1)
        a2 = _softplus(q2 - d2)
        a3 = _softplus(q3 - d3)
        cols.extend([a1, a2, a3, a1 * a1, a2 * a2, a3 * a3, a1 * a2, a1 * a3, a2 * a3])
        if degree >= 3:
            cols.extend([a1 ** 3, a2 ** 3, a3 ** 3])
        return np.column_stack(cols)
    if degree >= 2:
        cols.extend([q1 * q1, q2 * q2, q3 * q3, q1 * q2, q1 * q3, q2 * q3])
    if degree >= 3:
        cols.extend([
            q1 ** 3, q2 ** 3, q3 ** 3,
            q1 * q1 * q2, q1 * q1 * q3, q2 * q2 * q1, q2 * q2 * q3, q3 * q3 * q1, q3 * q3 * q2,
            q1 * q2 * q3,
        ])
    return np.column_stack(cols)


def _rbf_kernel(dist, kernel, epsilon):
    if kernel == 'thin_plate':
        return np.where(dist > 1e-12, dist * dist * np.log(np.maximum(dist, 1e-12)), 0.0)
    if kernel == 'multiquadric':
        return np.sqrt(dist * dist + epsilon * epsilon)
    if kernel == 'gaussian':
        return np.exp(-((dist / epsilon) ** 2))
    if kernel == 'cubic':
        return dist ** 3
    raise ValueError(f'Unknown RBF kernel: {kernel}')


class ForwardKinematicsModel:
    """Unified pure-NumPy forward kinematics model: q (3,) -> (tip_tangent (3,), centerline (21,3))."""

    def __init__(self, kind='rbf', *, degree=2, deadzone_mm=None, kernel='multiquadric', epsilon=2.5, reg=1e-2):
        self.kind = kind
        self.degree = degree
        self.deadzone_mm = tuple(deadzone_mm) if deadzone_mm is not None else None
        self.kernel = kernel
        self.epsilon = float(epsilon)
        self.reg = float(reg)
        self.centers = None
        self.weights = None  # shape (M, 66): [3 tangent + 63 flattened centerline]

    @property
    def name(self):
        if self.kind == 'poly':
            return f'poly_deg{self.degree}_reg{self.reg:g}'
        if self.kind == 'deadzone_poly':
            return f'deadzone_poly_deg{self.degree}_reg{self.reg:g}'
        return f'rbf_{self.kernel}_eps{self.epsilon:g}_reg{self.reg:g}'

    def _design_matrix(self, X):
        X = np.atleast_2d(np.asarray(X, dtype=np.float64))
        if self.kind == 'poly':
            return _poly_features(X, degree=self.degree)
        if self.kind == 'deadzone_poly':
            return _poly_features(X, degree=self.degree, deadzone_mm=self.deadzone_mm or (2.5, 2.5, 2.0))
        if self.kind == 'rbf':
            diff = X[:, None, :] - self.centers[None, :, :]
            dist = np.linalg.norm(diff, axis=2)
            phi = _rbf_kernel(dist, self.kernel, self.epsilon)
            poly = _poly_features(X, degree=1)
            return np.hstack((phi, poly))
        raise ValueError(f'Unknown model kind: {self.kind}')

    def fit(self, X, tangents, centerlines):
        X = np.asarray(X, dtype=np.float64)
        tangents = np.asarray(tangents, dtype=np.float64)
        centerlines = np.asarray(centerlines, dtype=np.float64).reshape(len(X), -1)
        Y = np.hstack((tangents, centerlines))  # (N, 66)

        if self.kind == 'rbf':
            # Use unique rounded actuation points as RBF centers to avoid collinearity across repeats.
            unique_map = {}
            for row in X:
                key = tuple(np.round(row, 1).tolist())
                unique_map.setdefault(key, []).append(row)
            self.centers = np.array([np.mean(v, axis=0) for v in unique_map.values()], dtype=np.float64)

        A = self._design_matrix(X)
        m = A.shape[1]
        reg_diag = np.full(m, self.reg, dtype=np.float64)
        if self.kind == 'rbf':
            # Do not penalize the constant intercept term in the polynomial tail.
            reg_diag[-4] = 0.0
        else:
            reg_diag[0] = 0.0
        lhs = A.T @ A + np.diag(reg_diag)
        rhs = A.T @ Y
        self.weights = np.linalg.solve(lhs, rhs)
        return self

    def predict(self, X):
        X = np.atleast_2d(np.asarray(X, dtype=np.float64))
        A = self._design_matrix(X)
        raw = A @ self.weights
        tan = raw[:, :3]
        norms = np.linalg.norm(tan, axis=1, keepdims=True)
        tan = tan / np.maximum(norms, 1e-9)
        curves = raw[:, 3:].reshape(len(X), 21, 3)
        # Keep root pinned at [0, 0, 0] in B_ref.
        curves = curves - curves[:, 0:1, :]
        return tan, curves

    def predict_tangent(self, q):
        tan, _ = self.predict(np.asarray(q, dtype=np.float64).reshape(1, 3))
        return tan[0]

    def to_dict(self):
        return dict(
            kind=self.kind,
            degree=self.degree,
            deadzone_mm=list(self.deadzone_mm) if self.deadzone_mm is not None else None,
            kernel=self.kernel,
            epsilon=self.epsilon,
            reg=self.reg,
            centers=self.centers.tolist() if self.centers is not None else None,
            weights=self.weights.tolist() if self.weights is not None else None,
        )

    @classmethod
    def from_dict(cls, data):
        model = cls(
            kind=data['kind'],
            degree=data.get('degree', 2),
            deadzone_mm=data.get('deadzone_mm'),
            kernel=data.get('kernel', 'multiquadric'),
            epsilon=data.get('epsilon', 2.5),
            reg=data.get('reg', 1e-2),
        )
        if data.get('centers') is not None:
            model.centers = np.asarray(data['centers'], dtype=np.float64)
        if data.get('weights') is not None:
            model.weights = np.asarray(data['weights'], dtype=np.float64)
        return model

    def save_json(self, path, metadata=None):
        payload = dict(model=self.to_dict(), metadata=metadata or {})
        Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')

    @classmethod
    def load_json(cls, path):
        payload = json.loads(Path(path).read_text(encoding='utf-8'))
        return cls.from_dict(payload['model']), payload.get('metadata', {})


def evaluate_predictions(true_tangents, pred_tangents, true_curves, pred_curves):
    """Compute angular error (deg), tip position error (mm), and 21-point centerline RMS (mm)."""
    t_true = np.asarray(true_tangents, dtype=np.float64)
    t_pred = np.asarray(pred_tangents, dtype=np.float64)
    dots = np.clip(np.sum(t_true * t_pred, axis=1), -1.0, 1.0)
    ang_err = np.degrees(np.arccos(dots))

    c_true = np.asarray(true_curves, dtype=np.float64)
    c_pred = np.asarray(pred_curves, dtype=np.float64)
    tip_err = np.linalg.norm(c_true[:, -1, :] - c_pred[:, -1, :], axis=1)
    curve_rms = np.sqrt(np.mean(np.sum((c_true - c_pred) ** 2, axis=2), axis=1))

    return dict(
        n=int(len(ang_err)),
        tangent_err_median_deg=float(np.median(ang_err)),
        tangent_err_mean_deg=float(np.mean(ang_err)),
        tangent_err_p90_deg=float(np.percentile(ang_err, 90)),
        tangent_err_max_deg=float(np.max(ang_err)),
        tip_err_median_mm=float(np.median(tip_err)),
        tip_err_mean_mm=float(np.mean(tip_err)),
        tip_err_max_mm=float(np.max(tip_err)),
        centerline_rms_median_mm=float(np.median(curve_rms)),
        centerline_rms_mean_mm=float(np.mean(curve_rms)),
    ), ang_err, tip_err, curve_rms


def grouped_loocv(model_factory, samples, zero_anchors=None):
    """Leave-one-target-point-out CV keeping all repeats of each target_mm in the same fold."""
    groups = defaultdict(list)
    for idx, s in enumerate(samples):
        groups[tuple(s['target_mm'])].append(idx)

    pred_t = np.zeros((len(samples), 3), dtype=np.float64)
    pred_c = np.zeros((len(samples), 21, 3), dtype=np.float64)

    for target_key, val_idxs in groups.items():
        val_set = set(val_idxs)
        tr_samples = [s for i, s in enumerate(samples) if i not in val_set]
        if zero_anchors:
            tr_samples = tr_samples + zero_anchors
        X_tr = np.array([s['actual_mm'] for s in tr_samples], dtype=np.float64)
        T_tr = np.array([s['tip_tangent'] for s in tr_samples], dtype=np.float64)
        C_tr = np.array([s['centerline_mm'] for s in tr_samples], dtype=np.float64)

        model = model_factory()
        model.fit(X_tr, T_tr, C_tr)

        X_val = np.array([samples[i]['actual_mm'] for i in val_idxs], dtype=np.float64)
        pt, pc = model.predict(X_val)
        for local_i, global_i in enumerate(val_idxs):
            pred_t[global_i] = pt[local_i]
            pred_c[global_i] = pc[local_i]

    true_t = np.array([s['tip_tangent'] for s in samples], dtype=np.float64)
    true_c = np.array([s['centerline_mm'] for s in samples], dtype=np.float64)
    metrics, _, _, _ = evaluate_predictions(true_t, pred_t, true_c, pred_c)
    return metrics


def _project_box_and_sum(q, lo, hi, max_total_mm):
    q = np.clip(np.asarray(q, dtype=np.float64), lo, hi)
    if max_total_mm is not None and float(np.sum(q)) > max_total_mm:
        # Project onto the simplex cap sum(q) <= max_total_mm within [lo, hi] via bisection.
        a, b = 0.0, float(np.max(q - lo))
        for _ in range(30):
            mid = 0.5 * (a + b)
            if float(np.sum(np.clip(q - mid, lo, hi))) > max_total_mm:
                a = mid
            else:
                b = mid
        q = np.clip(q - b, lo, hi)
    return q


def solve_inverse_tangent(
    model,
    target_tangent,
    *,
    q_init=None,
    bounds=(0.0, 6.0),
    max_total_mm=12.0,
    move_reg=1e-4,
    active_axes=None,
):
    """Solve for q in [lo, hi]^3 minimizing ||t_hat(q) - t*||^2 + move_reg * ||q - q_init||^2.

    Uses multi-start projected Levenberg-Marquardt in pure NumPy.
    If active_axes (e.g. [1, 2]) is given, inactive axes are held fixed at 0.
    """
    t_star = np.asarray(target_tangent, dtype=np.float64).ravel()
    t_star = t_star / float(np.linalg.norm(t_star))
    lo = np.full(3, float(bounds[0]), dtype=np.float64)
    hi = np.full(3, float(bounds[1]), dtype=np.float64)
    if active_axes is not None:
        active_set = set(active_axes)
        for i in range(3):
            if (i + 1) not in active_set:
                lo[i] = 0.0
                hi[i] = 0.0

    q_ref = np.zeros(3, dtype=np.float64) if q_init is None else np.asarray(q_init, dtype=np.float64)
    starts = [
        _project_box_and_sum(q_ref, lo, hi, max_total_mm),
        _project_box_and_sum(0.5 * (lo + hi), lo, hi, max_total_mm),
    ]
    # Add coarse grid corners for reliable global convergence on deadzone landscapes.
    for v1 in (0.0, 3.5, 5.5):
        for v2 in (0.0, 3.5, 5.5):
            for v3 in (0.0, 3.5, 5.5):
                cand = _project_box_and_sum([v1, v2, v3], lo, hi, max_total_mm)
                if not any(np.allclose(cand, s, atol=0.3) for s in starts):
                    starts.append(cand)

    best_q = starts[0]
    best_cost = float('inf')
    eps = 1e-3

    for start in starts:
        q = start.copy()
        lam = 1e-2
        for _ in range(35):
            t_cur = model.predict_tangent(q)
            r_t = t_cur - t_star
            r_q = math.sqrt(move_reg) * (q - q_ref)
            cost = float(np.dot(r_t, r_t) + np.dot(r_q, r_q))

            # Finite-difference Jacobian J (3x3)
            J = np.zeros((3, 3), dtype=np.float64)
            for j in range(3):
                if hi[j] <= lo[j]:
                    continue
                dq = np.zeros(3, dtype=np.float64)
                dq[j] = eps
                t_plus = model.predict_tangent(np.clip(q + dq, lo, hi))
                t_minus = model.predict_tangent(np.clip(q - dq, lo, hi))
                step = min(hi[j], q[j] + eps) - max(lo[j], q[j] - eps)
                if step > 1e-9:
                    J[:, j] = (t_plus - t_minus) / step

            H = J.T @ J + (move_reg + lam) * np.eye(3)
            g = J.T @ r_t + move_reg * (q - q_ref)
            try:
                step_q = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                break
            q_new = _project_box_and_sum(q + step_q, lo, hi, max_total_mm)
            t_new = model.predict_tangent(q_new)
            r_t_new = t_new - t_star
            r_q_new = math.sqrt(move_reg) * (q_new - q_ref)
            new_cost = float(np.dot(r_t_new, r_t_new) + np.dot(r_q_new, r_q_new))
            if new_cost < cost:
                q = q_new
                lam = max(lam * 0.5, 1e-6)
                if abs(cost - new_cost) < 1e-9:
                    break
            else:
                lam = min(lam * 5.0, 1e4)

        t_final = model.predict_tangent(q)
        final_cost = float(np.sum((t_final - t_star) ** 2) + move_reg * np.sum((q - q_ref) ** 2))
        if final_cost < best_cost:
            best_cost = final_cost
            best_q = q

    t_pred = model.predict_tangent(best_q)
    ang_err_deg = float(math.degrees(math.acos(float(np.clip(np.dot(t_pred, t_star), -1.0, 1.0)))))
    return dict(q_mm=best_q.tolist(), predicted_tangent=t_pred.tolist(), angular_error_deg=ang_err_deg)
