"""Hardware-free unit tests for stereo 3D centerline reconstruction and NAS batch processing."""

from contextlib import redirect_stdout
import csv
import io
import json
import math
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from scripts import process_nas_dual_camera as nas
from stereo_reconstruction import StereoRig, deproject_d405_numpy, reconstruct_stereo_centerline


# Realistic calibration matching the 2026-11-25 D405 + Ace Pro rig geometry.
SAMPLE_CALIB = {
    'board': {'inner_corners': [7, 5], 'square_mm': 10.0},
    'd405': {
        'image_size': [640, 480],
        'camera_matrix': [[389.0, 0.0, 314.0], [0.0, 388.6, 240.3], [0.0, 0.0, 1.0]],
        'factory_intrinsics': {
            'width': 640,
            'height': 480,
            'fx': 389.0,
            'fy': 388.6,
            'ppx': 314.0,
            'ppy': 240.3,
            'model_id': 2,
            'model': 'distortion.inverse_brown_conrady',
            'coeffs': [-0.054, 0.057, -0.0012, 0.0001, -0.018],
        },
        'stereo_dist_coeffs': [0.0, 0.0, 0.0, 0.0, 0.0],
        'stereo_points': 'undistorted_pixels',
    },
    'side_view': {
        'image_size': [1920, 1080],
        'camera_matrix': [[1025.0, 0.0, 907.3], [0.0, 1008.8, 210.8], [0.0, 0.0, 1.0]],
        'dist_coeffs': [-0.087, -0.002, -0.033, -0.027, 0.015],
        'intrinsic_views': 194,
    },
    'R_d405_to_side': [
        [-0.03357314266463152, -0.9976818635797015, -0.05919242498626215],
        [-0.3183359171825094, -0.04546639973380362, 0.946887031449273],
        [-0.9473832845914008, 0.0506330482923944, -0.3160715211742893],
    ],
    't_d405_to_side_mm': [[41.826], [-137.050], [209.136]],
    'baseline_mm': 253.52,
    'stereo_rms_px': 2.30,
}


def _realsense_distort(undist_px, matrix, coeffs):
    """Apply librealsense modified/inverse Brown-Conrady forward distortion."""
    cx, cy = matrix[0, 2], matrix[1, 2]
    fx, fy = matrix[0, 0], matrix[1, 1]
    k1, k2, p1, p2, k3 = coeffs
    x = (undist_px[:, 0] - cx) / fx
    y = (undist_px[:, 1] - cy) / fy
    r2 = x * x + y * y
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xr, yr = x * radial, y * radial
    xd = xr + 2.0 * p1 * xr * yr + p2 * (r2 + 2.0 * xr * xr)
    yd = yr + 2.0 * p2 * xr * yr + p1 * (r2 + 2.0 * yr * yr)
    return np.column_stack((xd * fx + cx, yd * fy + cy))


def _project_rig(rig, pts_3d):
    """Project 3D points (mm in D405 frame) into distorted D405 and Ace Pro pixels."""
    uv_d_undist, _ = rig.project_undistorted(pts_3d)
    uv_d = _realsense_distort(uv_d_undist, rig.d405_K, rig.d405_coeffs)
    rvec, _ = cv2.Rodrigues(rig.R_d405_to_side)
    uv_s, _ = cv2.projectPoints(pts_3d, rvec, rig.t_d405_to_side_mm, rig.side_K, rig.side_dist)
    return uv_d, uv_s.reshape(-1, 2)


def _synthetic_catheter_3d(bend_deg, length_mm=30.0, n_pts=41):
    """Generate a smooth 3D circular arc catheter in D405 camera coordinates (mm)."""
    s = np.linspace(0.0, length_mm, n_pts)
    root = np.array([-10.0, -5.0, 200.0])
    if abs(bend_deg) < 1e-4:
        pts = root + np.column_stack((s, np.zeros_like(s), np.zeros_like(s)))
    else:
        theta = math.radians(bend_deg)
        radius = length_mm / theta
        ang = s / radius
        # Bend in a 3D plane tilted between Y and Z so both cameras see curvature.
        dx = radius * np.sin(ang)
        perp = radius * (1.0 - np.cos(ang))
        dy = perp * math.cos(math.radians(35.0))
        dz = perp * math.sin(math.radians(35.0))
        pts = root + np.column_stack((dx, dy, dz))
    return pts


def _render_synthetic_pair(width, height, angle_deg, *, is_d405=False):
    """Render an orange-shaft + white-tip synthetic catheter on a green backdrop."""
    image = np.full((height, width, 3), (60, 145, 55), dtype=np.uint8)
    cx, cy = width // 2, height // 2
    scale = width / 640.0
    # Orange proximal shaft
    shaft_len = int(round(45 * scale))
    tube_r = max(4, int(round(9 * scale)))
    cv2.line(image, (cx - shaft_len - 10, cy), (cx - 4, cy), (25, 130, 245), thickness=2 * tube_r)
    # White curved flexible tip (~30 mm at 200 mm depth with fx=389 -> ~58 px)
    tip_len = int(round(58 * scale))
    rad = math.radians(angle_deg)
    pts = []
    for t in np.linspace(0.0, 1.0, 25):
        if abs(rad) < 1e-4:
            x = cx + t * tip_len
            y = cy
        else:
            r = tip_len / rad
            x = cx + r * math.sin(t * rad)
            y = cy + r * (1.0 - math.cos(t * rad))
        pts.append((int(round(x)), int(round(y))))
    for a, b in zip(pts, pts[1:]):
        cv2.line(image, a, b, (240, 240, 240), thickness=2 * tube_r)

    if not is_d405:
        return image

    # Depth in uint16 (0.1 mm units): background at 260 mm, catheter at 200 mm.
    depth = np.full((height, width), 2600, dtype=np.uint16)
    cv2.line(depth, (cx - shaft_len - 10, cy), (cx - 4, cy), 2000, thickness=2 * tube_r)
    for a, b in zip(pts, pts[1:]):
        cv2.line(depth, a, b, 2000, thickness=2 * tube_r)
    return image, depth


class StereoReconstructionTests(unittest.TestCase):
    def test_pure_numpy_d405_deprojection_inverts_realsense_distortion(self):
        intr = SAMPLE_CALIB['d405']['factory_intrinsics']
        matrix = np.array(SAMPLE_CALIB['d405']['camera_matrix'])
        coeffs = np.array(intr['coeffs'])
        pt3d = np.array([[22.5, -14.0, 210.0]])
        undist = np.array([[pt3d[0, 0] / pt3d[0, 2] * intr['fx'] + intr['ppx'],
                            pt3d[0, 1] / pt3d[0, 2] * intr['fy'] + intr['ppy']]])
        distorted = _realsense_distort(undist, matrix, coeffs)[0]
        recovered = deproject_d405_numpy(distorted, 210.0, intr)
        np.testing.assert_allclose(recovered, pt3d[0], atol=0.05)

    def test_stereo_reconstruction_recovers_3d_curve_and_bend_angle(self):
        rig = StereoRig.from_dict(SAMPLE_CALIB)
        for true_bend in (0.0, 25.0, 50.0):
            with self.subTest(true_bend=true_bend):
                curve_3d = _synthetic_catheter_3d(true_bend, length_mm=30.0, n_pts=21)
                uv_d, uv_s = _project_rig(rig, curve_3d)
                res = reconstruct_stereo_centerline(
                    rig, uv_d.tolist(), uv_s.tolist(), length_mm=30.0, shaft_direction=[1.0, 0.0, 0.0]
                )
                self.assertTrue(res['valid'], res['reason'])
                self.assertLess(res['reproj_rms_px'], 0.5)
                self.assertAlmostEqual(res['length_mm'], 30.0, delta=0.5)
                rec = np.asarray(res['centerline_camera_mm'])
                self.assertEqual(rec.shape, (21, 3))
                errs = np.linalg.norm(rec - curve_3d, axis=1)
                self.assertLess(float(errs.mean()), 0.5)
                self.assertLess(float(errs.max()), 1.5)
                self.assertLess(float(errs[-1]), 0.2)
                # fit_distal_tangent evaluates at 85% arc length.
                expected_85 = true_bend * 0.85
                self.assertAlmostEqual(res['angle_vs_d405_shaft_deg'], expected_85, delta=1.5)
                self.assertAlmostEqual(res['angle_deg'], expected_85, delta=4.0)


class NasPipelineTests(unittest.TestCase):
    def _create_batch_tar(self, tar_path, ids, target_mm, repeat=1):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'batch'
            round_name = '20261125_150000_test0001'
            rdir = root / round_name
            sdir = rdir / 'side_view'
            sdir.mkdir(parents=True)

            (root / 'points.json').write_text(json.dumps(dict(
                ids=ids,
                points_mm=[target_mm],
                repeats=1,
                reference_mm=[0, 0, 0],
                completed_rounds=[dict(point=1, repeat=repeat, path=round_name)],
            )), encoding='utf-8')
            # Add macOS AppleDouble junk file to verify safe skipping.
            (root / '._points.json').write_bytes(b'\x00\x05\x16\x07\xa3invalid-utf8')

            events = dict(reference_hold=100.0, T0=102.0, target_hold=105.0, **{'return': 109.0}, T2=112.0)
            stamps = [
                (100.5, 0.0, [0.0, 0.0, 0.0]),
                (101.2, 0.0, [0.0, 0.0, 0.0]),
                (103.5, 12.0, [v * 0.5 for v in target_mm]),
                (106.0, 28.0, [float(v) for v in target_mm]),
                (107.5, 28.0, [float(v) for v in target_mm]),
                (110.2, 14.0, [v * 0.5 for v in target_mm]),
                (111.7, 0.5, [0.0, 0.0, 0.0]),
            ]

            d405_lines = []
            side_lines = []
            for idx, (stamp, angle, pos) in enumerate(stamps):
                color, depth = _render_synthetic_pair(640, 480, angle, is_d405=True)
                npz_name = f'frame_{idx:06d}.npz'
                np.savez_compressed(rdir / npz_name, color_bgr=color, native_depth=depth, aligned_depth=depth)
                axes = [dict(id=i, position=pos[i - 1], online=True, error='', last_seen=stamp - 0.01) for i in (1, 2, 3)]
                d405_lines.append(dict(
                    kind='frame',
                    file=npz_name,
                    frame_number=idx,
                    host_receive_monotonic=stamp,
                    telemetry=dict(valid=True, axes=axes),
                    measurement=dict(valid=True),
                ))

                side_bgr = _render_synthetic_pair(640, 360, angle, is_d405=False)
                png_name = f'frame_{idx:06d}.png'
                self.assertTrue(cv2.imwrite(str(sdir / png_name), side_bgr))
                side_lines.append(dict(
                    kind='frame',
                    file=png_name,
                    frame_number=idx,
                    host_receive_monotonic=stamp + 0.015,
                    telemetry=dict(valid=True, axes=axes),
                    measurement=dict(valid=True),
                ))

            for kind, t in events.items():
                d405_lines.append(dict(kind='annotation', host_monotonic=t, note=dict(kind=kind)))
            d405_lines.append(dict(kind='end', frames=len(stamps), reason='本轮结束'))
            side_lines.append(dict(kind='end', frames=len(stamps), reason='本轮结束'))

            (rdir / 'frames.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in d405_lines), encoding='utf-8')
            (sdir / 'frames.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in side_lines), encoding='utf-8')
            (rdir / 'metadata.json').write_text(json.dumps(dict(
                settings=dict(roi=[0.0, 0.0, 1.0, 1.0], white_min=85, white_sat=100, length_mm=30.0, diameter_mm=6.0),
                calibration=dict(depth_scale_m=0.0001, color=SAMPLE_CALIB['d405']['factory_intrinsics']),
            )), encoding='utf-8')
            (sdir / 'metadata.json').write_text(json.dumps(dict(
                settings=dict(roi=[0.0, 0.0, 1.0, 1.0], white_min=85, white_sat=100,
                              orange_min=[3, 85, 55], orange_max=[27, 255, 255]),
                format=dict(width=640, height=360, fps=30.0),
            )), encoding='utf-8')
            (rdir / 'round.json').write_text(json.dumps(dict(
                status='complete',
                point=1,
                repeat=repeat,
                ids=ids,
                target_mm=target_mm,
                reference_mm=[0, 0, 0],
                frame_count=len(stamps),
                side_view=dict(frame_count=len(stamps)),
            )), encoding='utf-8')

            tar_path.parent.mkdir(parents=True, exist_ok=True)
            with tarfile.open(tar_path, 'w') as tf:
                for item in sorted(root.rglob('*')):
                    tf.add(item, arcname='./' + str(item.relative_to(root)))

    def test_end_to_end_dry_run_process_resume_and_tip_leak_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            nas_root = Path(tmp) / 'nas'
            out_root = nas_root / 'OPHIS_双机全量分析_2026-11-25'
            calib_path = nas_root / 'OPHIS_双目标定_2026-11-25' / 'stereo_calibration.json'
            calib_path.parent.mkdir(parents=True)
            calib_path.write_text(json.dumps(SAMPLE_CALIB), encoding='utf-8')

            self._create_batch_tar(
                nas_root / 'OPHIS_双机单轴位移采集_1-10mm_2026-11-25' / 'ID1_01-05mm.tar',
                ids=[1],
                target_mm=[4, 0, 0],
            )
            self._create_batch_tar(
                nas_root / 'OPHIS_双机双轴验证_3-5mm_2026-11-25' / 'ID2_ID3_03-05mm_tip_leak.tar',
                ids=[2, 3],
                target_mm=[0, 3, 5],
            )

            # 1. Dry run should not create output directory.
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = nas.main(['--nas-root', str(nas_root), '--dry-run'])
            self.assertEqual(rc, 0)
            self.assertFalse(out_root.exists())
            self.assertIn('[标记:tip_leak]', buf.getvalue())

            # 2. Full run without pyrealsense2 (tests pure-NumPy fallback + tar extraction + aggregation).
            buf = io.StringIO()
            with redirect_stdout(buf), patch.dict(sys.modules, {'pyrealsense2': None}):
                rc = nas.main(['--nas-root', str(nas_root), '--workers', '1'])
            self.assertEqual(rc, 0)
            self.assertTrue((out_root / 'all_rounds.csv').is_file())
            self.assertTrue((out_root / 'all_points.csv').is_file())
            self.assertTrue((out_root / 'single_axis_resolution.csv').is_file())
            self.assertTrue((out_root / 'summary_report.json').is_file())

            report = json.loads((out_root / 'summary_report.json').read_text(encoding='utf-8'))
            self.assertEqual(report['completed_batches'], 2)
            self.assertEqual(report['total_rounds'], 2)
            self.assertEqual(report['clean_rounds'], 1)
            self.assertEqual(report['flagged_rounds'], 1)

            with (out_root / 'all_rounds.csv').open(encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            by_slug = {r['round_slug']: r for r in rows}
            self.assertIn('ID1_04mm_R01', by_slug)
            self.assertIn('ID2_03mm_ID3_05mm_R01', by_slug)
            self.assertEqual(by_slug['ID1_04mm_R01']['quality_flag'], 'ok')
            self.assertEqual(by_slug['ID2_03mm_ID3_05mm_R01']['quality_flag'], 'tip_leak')
            self.assertGreater(int(by_slug['ID1_04mm_R01']['d405_hold_valid']), 0)
            self.assertGreater(int(by_slug['ID1_04mm_R01']['side_hold_valid']), 0)
            self.assertEqual(int(by_slug['ID1_04mm_R01']['stereo_paired_frames']), 7)

            # 3. Second run without --force should skip both completed batches.
            buf2 = io.StringIO()
            with redirect_stdout(buf2), patch.dict(sys.modules, {'pyrealsense2': None}):
                rc2 = nas.main(['--nas-root', str(nas_root), '--workers', '1'])
            self.assertEqual(rc2, 0)
            self.assertEqual(buf2.getvalue().count('[跳过已完成]'), 2)


if __name__ == '__main__':
    unittest.main()
