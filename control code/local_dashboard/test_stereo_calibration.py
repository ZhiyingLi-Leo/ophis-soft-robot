"""Hardware-free calibration and capture tests using a synthetic stereo rig."""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from scripts import capture_checkerboard
from stereo_calibration import (
    DetectedPair, Detection, Frame, InsufficientPairsError, _orient_side_points, calibrate_session, collapse_poses,
    detect_checkerboard, pair_frames,
)


D405_SIZE = (640, 480)
SIDE_SIZE = (1920, 1080)
D405_K = np.array([[389., 0., 314.], [0., 388., 240.], [0., 0., 1.]])
SIDE_K = np.array([[985., 0., 965.], [0., 975., 535.], [0., 0., 1.]])
D405_DIST = np.array([-.054, .057, -.0012, .0001, -.018])
SIDE_DIST = np.array([-.08, .035, .0009, -.0007, -.006])
SIDE_ROLL = cv2.Rodrigues(np.array([0., 0., math.radians(40)]))[0]
R_GT = SIDE_ROLL @ cv2.Rodrigues(np.array([math.pi / 2, 0., 0.]))[0]
T_GT = SIDE_ROLL @ np.array([24., 320., 340.])


def pixel_map(size, matrix, distortion, *, realsense=False):
    """Map each distorted output pixel to a 2× oversampled ideal image."""
    pixels = np.indices(size[::-1], dtype=np.float64)[::-1].reshape(2, -1).T
    if realsense:
        focal, center = matrix.diagonal()[:2], matrix[:2, 2]
        target = (pixels - center) / focal
        ideal = target.copy()
        k1, k2, p1, p2, k3 = distortion
        for _ in range(14):
            r2 = np.sum(ideal**2, axis=1)
            radial = 1 + k1*r2 + k2*r2**2 + k3*r2**3
            xr, yr = (ideal * radial[:, None]).T
            forward = np.column_stack((xr + 2*p1*xr*yr + p2*(r2 + 2*xr**2),
                                       yr + 2*p2*xr*yr + p1*(r2 + 2*yr**2)))
            ideal += target - forward
        undistorted = ideal * focal + center
    else:
        undistorted = cv2.undistortPoints(pixels.reshape(-1, 1, 2), matrix, distortion, P=matrix)
    return (undistorted * 2).reshape(size[1], size[0], 2).astype(np.float32)


def render_board(rotation, center, matrix, size, lookup):
    square_px = 80
    source = np.full((8 * square_px, 10 * square_px), 245, dtype=np.uint8)
    for row in range(6):
        for col in range(8):
            if (row + col) % 2 == 0:
                y, x = (row + 1) * square_px, (col + 1) * square_px
                source[y:y + square_px, x:x + square_px] = 14
    outside = np.float32([[-50, -40, 0], [50, -40, 0], [50, 40, 0], [-50, 40, 0]])
    rvec = cv2.Rodrigues(rotation)[0]
    projected, _ = cv2.projectPoints(outside, rvec, center, matrix, None)
    assert np.all(projected > 10) and np.all(projected < (size[0]-10, size[1]-10)), projected
    height, width = source.shape
    boundary = np.float32([[-.5, -.5], [width-.5, -.5], [width-.5, height-.5], [-.5, height-.5]])
    warp = cv2.getPerspectiveTransform(boundary, projected.reshape(-1, 2) * 2)
    ideal = cv2.warpPerspective(source, warp, (size[0]*2, size[1]*2), borderValue=145)
    blurred = cv2.GaussianBlur(ideal, (7, 7), 1.5)
    return cv2.remap(blurred, lookup, None, cv2.INTER_LINEAR, borderValue=145)


class SessionWriter:
    def __init__(self, directory):
        self.directory = directory
        self.side = directory / 'side_view'
        self.side.mkdir()
        self.rows = {directory: [], self.side: []}
        color = dict(width=D405_SIZE[0], height=D405_SIZE[1], fx=D405_K[0, 0], fy=D405_K[1, 1],
                     ppx=D405_K[0, 2], ppy=D405_K[1, 2], model_id=2,
                     model='distortion.inverse_brown_conrady', coeffs=D405_DIST.tolist())
        (directory / 'metadata.json').write_text(json.dumps(dict(
            label='checkerboard', calibration=dict(color=color), target_record_hz=5)), encoding='utf-8')
        (self.side / 'metadata.json').write_text(json.dumps(dict(
            name='Ace Pro', format=dict(width=SIDE_SIZE[0], height=SIDE_SIZE[1], fps=30),
            target_record_hz=5)), encoding='utf-8')

    def frame(self, camera, gray, stamp):
        path = self.directory if camera == 'd405' else self.side
        index = len(self.rows[path])
        name = f'frame_{index:06d}.npz' if camera == 'd405' else f'frame_{index:06d}.png'
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if camera == 'd405':
            np.savez_compressed(path / name, color_bgr=bgr, native_depth=np.zeros(gray.shape, np.uint16))
        else:
            if not cv2.imwrite(str(path / name), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError(name)
        self.rows[path].append(dict(kind='frame', file=name, host_receive_monotonic=stamp,
                                    frame_number=900 + 6*index, measurement=dict(valid=False)))
        return name

    def finish(self):
        for path, frames in self.rows.items():
            lines = [*frames, dict(kind='end', frames=len(frames), reason='本轮结束')]
            (path / 'frames.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in lines),
                                             encoding='utf-8')


class StereoCalibrationTests(unittest.TestCase):
    def test_greedy_pairing_prioritizes_closest_and_is_one_to_one(self):
        def frames(*timestamps):
            return [Frame(str(stamp), stamp, None) for stamp in timestamps]

        d405 = frames(101, 100.0, 100.06, 102.01, 103)
        side = frames(100.05, 99.91, 101.12, 102.131)
        matched = pair_frames(d405, side)
        self.assertEqual([(a.file, b.file) for a, b in matched],
                         [('100.0', '99.91'), ('100.06', '100.05'), ('101', '101.12')])

    def test_duplicate_pose_selection_uses_sharpness_and_then_time_gap(self):
        points = np.float32([[1, 2], [19, 6], [42, 12]])
        a = Detection(points, 30)
        b = Detection(points[::-1], 30)
        low_sharpness = Detection(points, 10)
        stream = [
            DetectedPair(Frame('a', 1, 1), Frame('as', 1.03, 5), a, a),
            DetectedPair(Frame('b', 1.2, 2), Frame('bs', 1.21, 6), b, a),
            DetectedPair(Frame('c', 1.4, 3), Frame('cs', 1.401, 7), low_sharpness, a),
            DetectedPair(Frame('d', 1.6, 4), Frame('ds', 1.61, 8), Detection(points + 20, 30), a),
        ]
        self.assertEqual([p.d405_frame.file for p in collapse_poses(stream)], ['b', 'd'])

    def test_classic_detector_fallback(self):
        gray = np.full((260, 340), 240, np.uint8)
        for row in range(6):
            for col in range(8):
                if (row + col) % 2 == 0:
                    y, x = 40 + 30*row, 50 + 30*col
                    gray[y:y + 30, x:x + 30] = 15
        with patch('stereo_calibration.cv2.findChessboardCornersSB', return_value=(False, None)), \
             patch('stereo_calibration.cv2.cornerSubPix', wraps=cv2.cornerSubPix) as refine:
            detection = detect_checkerboard(gray)
        refine.assert_called_once()
        self.assertEqual(detection.corners.shape, (35, 2))
        np.testing.assert_allclose(detection.corners[[0, -1]], [[79.5, 69.5], [259.5, 189.5]], atol=.05)

    def test_rotating_at_one_center_in_one_plane_does_not_resolve_half_turn(self):
        board = np.float32([(x, y, 0) for y in range(0, 41, 10) for x in range(0, 61, 10)])
        center = np.array([0., 0., 330.])
        tilt = cv2.Rodrigues(np.array([-math.pi / 4, 0., 0.]))[0]
        left, right = [], []
        for angle in np.linspace(0, math.pi, 8):
            rotation = tilt @ cv2.Rodrigues(np.array([0., 0., angle]))[0]
            origin = center - rotation @ (board[-1] / 2)
            left.append(cv2.projectPoints(board, cv2.Rodrigues(rotation)[0], origin, D405_K, None)[0]
                        .reshape(-1, 2))
            right.append(cv2.projectPoints(board, cv2.Rodrigues(R_GT @ rotation)[0],
                                           R_GT @ origin + T_GT, SIDE_K, None)[0].reshape(-1, 2))
        with self.assertRaisesRegex(ValueError, '180° 朝向无法确定'):
            _orient_side_points(board, left, right, D405_K, None, SIDE_K, None)

    def test_twelve_pose_stereo_calibration(self):
        poses = [
            (-100, -55, 256, -42, 6, 18), (89, 54, 355, -57, -12, -27),
            (-56, 75, 402, -35, 10, 102), (76, -57, 288, -49, 12, 160),
            (0, 8, 325, -56, 15, -74), (-86, 54, 290, -29, 4, -128),
            (96, -29, 380, -40, -16, 45), (14, 86, 362, -54, -5, -12),
            (-88, -33, 355, -30, 6, 73), (76, 37, 267, -59, -11, 120),
            (15, -68, 310, -39, 8, -57), (-35, 51, 402, -50, -16, 157),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            writer = SessionWriter(directory)
            d405_lookup = pixel_map(D405_SIZE, D405_K, D405_DIST, realsense=True)
            side_lookup = pixel_map(SIDE_SIZE, SIDE_K, SIDE_DIST)
            keep = []
            for i, (x, y, z, rx, ry, rz) in enumerate(poses):
                angles = np.deg2rad([rx, ry, rz])
                rx_m, ry_m, rz_m = [cv2.Rodrigues(np.eye(3)[j] * angle)[0]
                                    for j, angle in enumerate(angles)]
                board_rotation = rx_m @ ry_m @ rz_m
                center = np.array([x, y, z], dtype=np.float64)
                d405 = render_board(board_rotation, center, D405_K, D405_SIZE, d405_lookup)
                side = render_board(R_GT @ board_rotation, R_GT @ center + T_GT,
                                    SIDE_K, SIDE_SIZE, side_lookup)
                stamp = 523400 + i
                keep.append(writer.frame('d405', d405, stamp))
                writer.frame('side', side, stamp + .023)
                writer.frame('d405', cv2.GaussianBlur(d405, (7, 7), 1.0), stamp + .21)
                writer.frame('side', cv2.GaussianBlur(side, (7, 7), 1.0), stamp + .215)
            writer.frame('side', side, 523430)
            writer.finish()

            result = calibrate_session(directory, square_mm=10)
            self.assertEqual(result['counts']['paired_frames'], 24)
            self.assertEqual(result['counts']['detected_pairs'], 24)
            self.assertEqual(result['counts']['selected_poses'], 12)
            self.assertEqual(result['counts']['d405_only'], 0)
            self.assertEqual(result['counts']['side_only'], 1)
            self.assertEqual(result['side_view']['intrinsic_views'], 25)
            self.assertEqual([p['d405']['file'] for p in result['selected_pairs']], keep)
            self.assertEqual({p['side_order_reversed'] for p in result['selected_pairs']}, {True, False})
            self.assertTrue(all(abs(p['dt_ms'] - 23) < 1e-5 for p in result['selected_pairs']))

            rotation = np.array(result['R_d405_to_side'])
            translation = np.array(result['t_d405_to_side_mm'])
            self.assertEqual(rotation.shape, (3, 3))
            self.assertEqual(translation.shape, (3, 1))
            angle_error = math.degrees(np.linalg.norm(cv2.Rodrigues(rotation @ R_GT.T)[0]))
            translation_error = np.linalg.norm(translation.ravel() - T_GT)
            self.assertLess(angle_error, .5)
            self.assertLess(translation_error, 1.5)
            self.assertLess(result['stereo_rms_px'], .5)
            self.assertAlmostEqual(result['baseline_mm'], np.linalg.norm(translation))
            self.assertEqual(result['board'], dict(inner_corners=[7, 5], square_mm=10))
            self.assertEqual(result['d405']['factory_intrinsics']['coeffs'], D405_DIST.tolist())
            np.testing.assert_allclose(result['d405']['camera_matrix'], D405_K)
            self.assertEqual(result['d405']['image_size'], list(D405_SIZE))
            self.assertEqual(result['side_view']['image_size'], list(SIDE_SIZE))
            self.assertLess(result['ace_pro_rms_px'], .5)
            path = directory / 'stereo_calibration.json'
            self.assertEqual(json.loads(path.read_text(encoding='utf-8')), result)

    def test_insufficient_detections_preserve_recording_and_report_each_camera(self):
        board = np.full((480, 640), 240, np.uint8)
        for row, col in np.ndindex(6, 8):
            if (row + col) % 2 == 0:
                y, x = 120 + 30*row, 160 + 30*col
                board[y:y + 30, x:x + 30] = 15
        side_board = cv2.copyMakeBorder(board, 300, 300, 640, 640, cv2.BORDER_CONSTANT, value=240)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            writer = SessionWriter(directory)
            saved = writer.frame('d405', board, 10)
            writer.frame('d405', np.full_like(board, 145), 11)
            writer.frame('side', np.full_like(side_board, 145), 10.01)
            writer.frame('side', side_board, 11.01)
            writer.finish()
            with self.assertRaises(InsufficientPairsError) as caught:
                calibrate_session(directory, square_mm=8)
            self.assertEqual(caught.exception.counts['paired_frames'], 2)
            self.assertEqual(caught.exception.counts['d405_only'], 1)
            self.assertEqual(caught.exception.counts['side_only'], 1)
            stdout = io.StringIO()
            with redirect_stdout(stdout), patch.object(capture_checkerboard, 'Robot') as robot:
                code = capture_checkerboard.main(['--session', tmp, '--square-mm', '8'])
            robot.assert_not_called()
            self.assertEqual(code, 1)
            self.assertIn('仅 D405 检出 1 帧，仅 Ace Pro 检出 1 帧', stdout.getvalue())
            self.assertIn('原始记录保留', stdout.getvalue())
            self.assertTrue((directory / saved).is_file())
            self.assertFalse((directory / 'stereo_calibration.json').exists())


class FakeRobot:
    def __init__(self, directory, *, fail_wait=False, fail_start=False):
        self.directory = directory
        self.fail_wait = fail_wait
        self.fail_start = fail_start
        self.recording = False
        self.side_age = .05
        self.clock = 0.
        self.events = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.events.append('close')

    def state(self):
        def camera(age):
            return dict(state=dict(online=True, error='', frame_age=age, recording=self.recording,
                                   finalizing=False, record_error=''))
        return dict(modules={'camera.d405': camera(.02), 'camera.side_view': camera(self.side_age)})

    def axis_record_start(self, label, group):
        self.events.append(('start', label, group))
        self.recording = True
        if self.fail_start:
            raise RuntimeError('录制响应超时')
        return dict(session_path=str(self.directory))

    def axis_record_stop(self):
        self.events.append('stop')
        self.recording = False

    def wait(self, seconds):
        if self.fail_wait:
            raise RuntimeError('录制期间断线')
        self.clock += seconds


class CaptureCheckerboardTests(unittest.TestCase):
    def test_live_capture_stops_before_calibrating_and_sends_no_actuator_commands(self):
        path = Path('/tmp/test_checkerboard_session')
        robot = FakeRobot(path)
        result = dict(counts=dict(paired_frames=26, detected_pairs=24, selected_poses=12,
                                  d405_only=1, side_only=1, ambiguous_pairs=0),
                      ace_pro_rms_px=.2, stereo_rms_px=.3, baseline_mm=465)
        stdout = io.StringIO()

        def calibrate(directory, square_mm, **options):
            self.assertEqual(robot.events[1:], ['stop', 'close'])
            self.assertFalse(robot.recording)
            self.assertEqual((directory, square_mm), (path, 10))
            self.assertEqual(options, dict(inner_corners=(7, 5), max_dt_ms=120, min_pairs=8))
            return result

        with (patch.dict('os.environ', OPHIS_URL='mock', OPHIS_TOKEN='mock'),
              patch.object(capture_checkerboard, 'Robot', return_value=robot),
              patch.object(capture_checkerboard.time, 'monotonic', side_effect=lambda: robot.clock),
              patch.object(capture_checkerboard, 'calibrate_session', side_effect=calibrate),
              redirect_stdout(stdout)):
            code = capture_checkerboard.main(['--square-mm', '10', '--seconds', '11'])
        self.assertEqual(code, 0)
        self.assertEqual(robot.events[0][:2], ('start', 'checkerboard'))
        self.assertRegex(robot.events[0][2], r'^stereo_calib_\d{8}_\d{6}_\d{6}$')
        self.assertEqual(stdout.getvalue().count('剩余'), 3)
        self.assertIn('立体重投影 RMS 0.300 px', stdout.getvalue())
        self.assertIn('stereo_calibration.json', stdout.getvalue())

    def test_capture_stops_on_error(self):
        for options, message in ((dict(fail_wait=True), '录制期间断线'),
                                 (dict(fail_start=True), '录制响应超时')):
            with self.subTest(options=options), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                robot = FakeRobot(Path('/tmp/test_checkerboard_session'), **options)
                with self.assertRaisesRegex(RuntimeError, message):
                    capture_checkerboard.capture_session(robot, seconds=2)
                self.assertEqual(robot.events[1:], ['stop'])
                self.assertFalse(robot.recording)

    def test_stale_or_recording_cameras_do_not_start_capture(self):
        for settings in (dict(side_age=1.1), dict(recording=True)):
            with self.subTest(settings=settings):
                robot = FakeRobot(Path('/tmp/test_checkerboard_session'))
                for key, value in settings.items():
                    setattr(robot, key, value)
                with self.assertRaises(RuntimeError):
                    capture_checkerboard.capture_session(robot, seconds=2)
                self.assertEqual(robot.events, [])


if __name__ == '__main__':
    unittest.main()
