"""Hardware-free unit tests for 3D forward kinematics modeling and inverse solving."""

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from kinematics_model import (
    ForwardKinematicsModel,
    build_reference_frame,
    evaluate_predictions,
    grouped_loocv,
    solve_inverse_tangent,
)


def _synthetic_arc_curve(theta_rad, phi_rad, length_mm=30.0, n_pts=21):
    s = np.linspace(0.0, length_mm, n_pts)
    if abs(theta_rad) < 1e-5:
        curve = np.column_stack((np.zeros(n_pts), np.zeros(n_pts), s))
        tan = np.array([0.0, 0.0, 1.0])
        return curve, tan
    r = length_mm / theta_rad
    ang = s / r
    radial = r * (1.0 - np.cos(ang))
    z = r * np.sin(ang)
    x = radial * math.cos(phi_rad)
    y = radial * math.sin(phi_rad)
    curve = np.column_stack((x, y, z))
    t85 = 0.85 * theta_rad
    tan = np.array([math.sin(t85) * math.cos(phi_rad), math.sin(t85) * math.sin(phi_rad), math.cos(t85)])
    return curve, tan / np.linalg.norm(tan)


def _synthetic_dataset():
    """Three-chamber deadzone + smooth bending synthetic dataset matching OPHIS geometry."""
    dirs = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]
    samples = []
    for q1 in (0, 2, 4, 6):
        for q2 in (0, 2, 4, 6):
            if q1 == 0 and q2 == 0:
                continue
            q = [float(q1), float(q2), 0.0]
            eff = [max(0.0, v - 2.0) for v in q]
            vx = sum(e * math.cos(a) for e, a in zip(eff, dirs))
            vy = sum(e * math.sin(a) for e, a in zip(eff, dirs))
            theta = math.radians(15.0 * math.hypot(vx, vy))
            phi = math.atan2(vy, vx)
            curve, tan = _synthetic_arc_curve(theta, phi)
            for rep in (1, 2):
                samples.append(dict(
                    target_mm=[q1, q2, 0],
                    actual_mm=q,
                    tip_tangent=tan,
                    centerline_mm=curve,
                    repeat=rep,
                ))
    return samples


class KinematicsModelTests(unittest.TestCase):
    def test_reference_frame_aligns_straight_curve_to_z(self):
        # Tilted 30 mm curve in camera coords
        s = np.linspace(0.0, 30.0, 21)
        direction = np.array([-0.1, 0.99, 0.1])
        direction /= np.linalg.norm(direction)
        ref_cam = np.array([-1.5, -2.0, 209.0]) + s[:, None] * direction[None, :]
        origin, R = build_reference_frame(ref_cam)
        local = (ref_cam - origin) @ R.T
        np.testing.assert_allclose(local[0], [0.0, 0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(local[-1], [0.0, 0.0, 30.0], atol=1e-9)

    def test_rbf_forward_model_json_roundtrip_and_inverse_solver(self):
        samples = _synthetic_dataset()
        X = np.array([s['actual_mm'] for s in samples])
        T = np.array([s['tip_tangent'] for s in samples])
        C = np.array([s['centerline_mm'] for s in samples])

        model = ForwardKinematicsModel('rbf', kernel='multiquadric', epsilon=2.0, reg=1e-3).fit(X, T, C)
        pt, pc = model.predict(X)
        metrics, _, _, _ = evaluate_predictions(T, pt, C, pc)
        self.assertLess(metrics['tangent_err_median_deg'], 1.0)
        self.assertLess(metrics['tip_err_median_mm'], 0.5)

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'forward_model.json'
            model.save_json(path, metadata={'note': 'test'})
            loaded, meta = ForwardKinematicsModel.load_json(path)
            self.assertEqual(meta['note'], 'test')
            pt2, pc2 = loaded.predict(X[:3])
            np.testing.assert_allclose(pt2, pt[:3], atol=1e-9)
            np.testing.assert_allclose(pc2, pc[:3], atol=1e-9)

        # Held-out target q=(5, 3, 0)
        q_true = np.array([5.0, 3.0, 0.0])
        t_star = model.predict_tangent(q_true)
        inv = solve_inverse_tangent(model, t_star, bounds=(0.0, 6.0), max_total_mm=12.0, active_axes=[1, 2])
        self.assertLess(inv['angular_error_deg'], 0.5)
        self.assertLess(float(np.linalg.norm(np.asarray(inv['q_mm']) - q_true)), 0.3)

    def test_grouped_loocv_runs_without_leakage(self):
        samples = _synthetic_dataset()
        metrics = grouped_loocv(
            lambda: ForwardKinematicsModel('rbf', kernel='thin_plate', reg=1e-2),
            samples,
        )
        self.assertEqual(metrics['n'], len(samples))
        self.assertLess(metrics['tangent_err_median_deg'], 5.0)


if __name__ == '__main__':
    unittest.main()
