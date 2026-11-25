"""Calibrate the D405 color and Ace Pro cameras from a paired recording."""

from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass
import json
from math import isfinite, sqrt
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np

DEFAULT_CORNERS = (7, 5)
DEFAULT_MAX_DT_MS = 120.0
DEFAULT_MIN_PAIRS = 8
POSE_DISTANCE_PX = 6.0


@dataclass(frozen=True)
class Frame:
    file: str
    host_receive_monotonic: float
    frame_number: int | None


@dataclass(frozen=True)
class Detection:
    corners: np.ndarray
    sharpness: float


@dataclass(frozen=True)
class DetectedPair:
    d405_frame: Frame
    side_frame: Frame
    d405: Detection
    side: Detection

    @property
    def delta_ms(self):
        return abs(self.d405_frame.host_receive_monotonic - self.side_frame.host_receive_monotonic) * 1000

    @property
    def quality(self):
        return sqrt(self.d405.sharpness * self.side.sharpness), -self.delta_ms


class InsufficientPairsError(ValueError):
    def __init__(self, counts, min_pairs):
        self.counts = counts
        self.min_pairs = min_pairs
        super().__init__(f"可用于立体标定的不同姿态只有 {counts['selected_poses']} 组，至少需要 {min_pairs} 组")


def read_frames(directory):
    with (directory / 'frames.jsonl').open(encoding='utf-8') as index:
        frames = []
        for line in index:
            item = json.loads(line)
            if item.get('kind') == 'frame':
                frame = Frame(item['file'], float(item['host_receive_monotonic']), item.get('frame_number'))
                if not isfinite(frame.host_receive_monotonic):
                    raise ValueError(f'{directory / frame.file} 的接收时间无效')
                frames.append(frame)
    return sorted(frames, key=lambda frame: frame.host_receive_monotonic)


def pair_frames(d405_frames, side_frames, max_dt_ms=DEFAULT_MAX_DT_MS):
    """Greedily take the globally closest remaining edge, then return pairs in time order."""
    d405_frames = sorted(d405_frames, key=lambda frame: frame.host_receive_monotonic)
    side_frames = sorted(side_frames, key=lambda frame: frame.host_receive_monotonic)
    side_times = [frame.host_receive_monotonic for frame in side_frames]
    tolerance = max_dt_ms / 1000 + 1e-9  # Allow floating-point roundoff at the boundary.
    candidates = []
    for i, frame in enumerate(d405_frames):
        stamp = frame.host_receive_monotonic
        start = bisect_left(side_times, stamp - tolerance)
        end = bisect_right(side_times, stamp + tolerance)
        candidates.extend((abs(stamp - side_times[j]), i, j) for j in range(start, end))

    matched_d405, matched_side, pairs = set(), set(), []
    for _, i, j in sorted(candidates):
        if i not in matched_d405 and j not in matched_side:
            matched_d405.add(i)
            matched_side.add(j)
            pairs.append((i, j))
    return [(d405_frames[i], side_frames[j]) for i, j in sorted(pairs)]


def detect_checkerboard(gray, inner_corners=DEFAULT_CORNERS):
    """Return a consistent grid winding and a deterministic image-local start corner."""
    found, corners = cv2.findChessboardCornersSB(gray, inner_corners, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gray, inner_corners, flags)
        if not found:
            return None

    cols, rows = inner_corners
    grid = corners.reshape(rows, cols, 2)
    # Refine both detectors at full resolution without including neighboring
    # corners in the gradient window when a projected square is small.
    spacing = min(np.linalg.norm(np.diff(grid, axis=axis), axis=2).min() for axis in (0, 1))
    window = int(np.clip(spacing / 3, 2, 6))
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .0001)
    grid = cv2.cornerSubPix(gray, corners, (window, window), (-1, -1), criteria).reshape(rows, cols, 2)
    dx = grid[0, -1] - grid[0, 0]
    dy = grid[-1, 0] - grid[0, 0]
    if dx[0] * dy[1] - dx[1] * dy[0] < 0:
        grid = grid[:, ::-1]
    corners = np.ascontiguousarray(grid.reshape(-1, 2))
    diagonal = corners[-1] - corners[0]
    if diagonal[np.argmax(np.abs(diagonal))] < 0:
        corners = np.ascontiguousarray(corners[::-1])

    hull = cv2.convexHull(np.rint(corners).astype(np.int32))
    x, y, width, height = cv2.boundingRect(hull)
    roi = gray[y:y + height, x:x + width]
    mask = np.zeros(roi.shape, np.uint8)
    cv2.fillConvexPoly(mask, hull - np.array([x, y], dtype=np.int32), 255)
    _, stddev = cv2.meanStdDev(cv2.Laplacian(roi, cv2.CV_32F), mask=mask)
    return Detection(corners, float(stddev[0, 0] ** 2))


def collapse_poses(pairs, distance_px=POSE_DISTANCE_PX):
    """Keep the sharpest pair per consecutive pose, breaking ties by timestamp gap."""
    selected = []
    anchor = None
    for pair in pairs:
        corners = pair.d405.corners
        if anchor is not None:
            displacement = min(np.linalg.norm(anchor - order, axis=1).mean()
                               for order in (corners, corners[::-1]))
            if displacement < distance_px:
                if pair.quality > selected[-1].quality:
                    selected[-1] = pair
                continue
        anchor = corners
        selected.append(pair)
    return selected


def _detect_frames(directory, frames, size, inner_corners, *, npz=False):
    detections = {}
    for frame in frames:
        path = directory / frame.file
        if npz:
            with np.load(path, allow_pickle=False) as data:
                gray = cv2.cvtColor(data['color_bgr'], cv2.COLOR_BGR2GRAY)
        else:
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError(f'无法读取图像：{path}')
        if gray.shape[::-1] != size:
            raise ValueError(f'{path} 的图像尺寸与记录的相机参数不一致')
        detection = detect_checkerboard(gray, inner_corners)
        if detection is not None:
            detections[frame] = detection
    return detections


def _d405_pinhole_points(corners, matrix, coeffs):
    # librealsense inverse/modified Brown–Conrady applies tangential terms after
    # radial scaling. Invert that projection to use fixed pinhole intrinsics in OpenCV.
    center = matrix[:2, 2]
    focal = matrix.diagonal()[:2]
    observed = (corners - center) / focal
    x, y = observed.T.copy()
    k1, k2, p1, p2, k3 = coeffs
    for _ in range(10):
        r2 = x*x + y*y
        radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xr, yr = x * radial, y * radial
        x = (observed[:, 0] - 2*p1*xr*yr - p2*(r2 + 2*xr*xr)) / radial
        y = (observed[:, 1] - 2*p2*xr*yr - p1*(r2 + 2*yr*yr)) / radial
    return (np.column_stack((x, y)) * focal + center).astype(np.float32)


def _board_pose(board, corners, matrix, distortion):
    ok, rvec, tvec = cv2.solvePnP(board, corners, matrix, distortion)
    if not ok:
        raise ValueError('棋盘格姿态求解失败，请增加双机共同可见的倾斜姿态')
    return cv2.Rodrigues(rvec)[0], tvec.ravel()


def _orient_side_points(board, d405_points, side_points, d405_matrix, d405_dist,
                        side_matrix, side_dist):
    # A plain 8×6 checkerboard is symmetric under a half-turn. Either indexing
    # gives an equally good monocular pose, but only one gives a fixed stereo rig
    # across distinct board poses. Score the rig hypotheses in side-view pixels.
    d405_boards, hypotheses = [], []
    half_turn = np.diag([-1., -1., 1.])
    for d405, side in zip(d405_points, side_points):
        rd, td = _board_pose(board, d405, d405_matrix, d405_dist)
        rs, ts = _board_pose(board, side, side_matrix, side_dist)
        d405_boards.append(board @ rd.T + td)
        for side_rotation, side_translation in ((rs, ts), (rs @ half_turn, ts + rs @ board[-1])):
            rotation = side_rotation @ rd.T
            hypotheses.append((rotation, side_translation - rotation @ td))

    camera_points = np.concatenate(d405_boards)
    side_points = np.array(side_points)
    alternatives = np.stack((side_points, side_points[:, ::-1]), axis=1)
    errors = []
    for rotation, translation in hypotheses:
        projected = cv2.projectPoints(camera_points, cv2.Rodrigues(rotation)[0],
                                      translation, side_matrix, side_dist)[0].reshape(side_points.shape)
        errors.append(np.sqrt(np.mean(np.sum((projected[:, None] - alternatives)**2, axis=-1), axis=-1)))

    scores = np.median(np.min(errors, axis=2), axis=1)
    best = int(np.argmin(scores))
    choices = np.argmin(errors, axis=2)
    reversed_majority = np.mean(choices != choices[best], axis=1) > .5
    if np.any(reversed_majority & (scores <= scores[best] + .5)):
        raise ValueError('棋盘格的 180° 朝向无法确定。请增加不同倾角与摆放位置后重试')

    # If even the closer ordering is more than halfway to its reversal, the
    # pair cannot establish a reliable correspondence (e.g. the board moved).
    best_error = np.min(errors[best], axis=1)
    separation = np.sqrt(np.mean(np.sum((alternatives[:, 0] - alternatives[:, 1])**2, axis=-1), axis=-1))
    return [None if error >= distance / 2 else bool(choice)
            for error, distance, choice in zip(best_error, separation, choices[best])]


def calibrate_session(session_dir, square_mm, *, inner_corners=DEFAULT_CORNERS,
                      max_dt_ms=DEFAULT_MAX_DT_MS, min_pairs=DEFAULT_MIN_PAIRS):
    """Save and return the fixed-intrinsic stereo calibration for one recording."""
    if not isfinite(square_mm) or square_mm <= 0:
        raise ValueError('棋盘格方格边长须为正数')
    if not isfinite(max_dt_ms) or max_dt_ms <= 0:
        raise ValueError('最大配对时间差须为正数')
    if min_pairs < 3:
        raise ValueError('最少姿态数须至少为 3')
    if min(inner_corners) < 3 or inner_corners[0] == inner_corners[1]:
        raise ValueError('棋盘格横纵内角点数须至少为 3 且不同')

    directory = Path(session_dir)
    side_dir = directory / 'side_view'
    factory = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))['calibration']['color']
    side_format = json.loads((side_dir / 'metadata.json').read_text(encoding='utf-8'))['format']
    d405_size = (factory['width'], factory['height'])
    side_size = (side_format['width'], side_format['height'])
    d405_frames, side_frames = read_frames(directory), read_frames(side_dir)
    pairs = pair_frames(d405_frames, side_frames, max_dt_ms)
    d405 = _detect_frames(directory, d405_frames, d405_size, inner_corners, npz=True)
    side = _detect_frames(side_dir, side_frames, side_size, inner_corners)
    detected = [DetectedPair(a, b, d405[a], side[b]) for a, b in pairs if a in d405 and b in side]
    selected = collapse_poses(detected)
    counts = dict(d405_frames=len(d405_frames), side_frames=len(side_frames), paired_frames=len(pairs),
                  detected_pairs=len(detected), selected_poses=len(selected),
                  d405_detections=len(d405), side_detections=len(side),
                  d405_only=len(d405) - len(detected), side_only=len(side) - len(detected),
                  ambiguous_pairs=0)
    if len(selected) < min_pairs:
        raise InsufficientPairsError(counts, min_pairs)

    cols, rows = inner_corners
    board = np.zeros((cols * rows, 3), np.float32)
    board[:, :2] = np.mgrid[:cols, :rows].T.reshape(-1, 2) * square_mm
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-8)
    side_rms, side_matrix, side_dist, _, _ = cv2.calibrateCamera(
        [board] * len(side), [d.corners for d in side.values()], side_size, None, None,
        flags=cv2.CALIB_USE_LU, criteria=criteria)

    d405_matrix = np.array([[factory['fx'], 0, factory['ppx']], [0, factory['fy'], factory['ppy']],
                            [0, 0, 1]], dtype=np.float64)
    d405_coeffs = np.array(factory['coeffs'], dtype=np.float64)
    model = factory.get('model', '').split('.')[-1]
    model_id = factory.get('model_id')
    undistort_d405 = model_id in (1, 2) or model in ('modified_brown_conrady', 'inverse_brown_conrady')
    d405_points = [pair.d405.corners for pair in selected]
    d405_dist = d405_coeffs
    if undistort_d405:
        d405_points = [_d405_pinhole_points(points, d405_matrix, d405_coeffs) for points in d405_points]
        d405_dist = np.zeros(5)
    elif np.any(d405_coeffs) and (model_id not in (None, 4) or model not in ('', 'brown_conrady')):
        raise ValueError(f'不支持的 D405 出厂畸变模型：{model or model_id}')

    side_points = [pair.side.corners for pair in selected]
    flips = _orient_side_points(board, d405_points, side_points, d405_matrix, d405_dist,
                                side_matrix, side_dist)
    counts['ambiguous_pairs'] = flips.count(None)
    counts['selected_poses'] -= counts['ambiguous_pairs']
    if counts['selected_poses'] < min_pairs:
        raise InsufficientPairsError(counts, min_pairs)

    used = [(pair, flip) for pair, flip in zip(selected, flips) if flip is not None]
    left = [points for points, flip in zip(d405_points, flips) if flip is not None]
    right = [np.ascontiguousarray(points[::-1] if flip else points)
             for points, flip in zip(side_points, flips) if flip is not None]
    # imageSize is used for intrinsic initialization, and both matrices are fixed.
    stereo_rms, _, _, _, _, rotation, translation, _, _ = cv2.stereoCalibrate(
        [board] * len(used), left, right, d405_matrix, d405_dist, side_matrix, side_dist,
        d405_size, criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)

    result = dict(
        board=dict(inner_corners=list(inner_corners), square_mm=square_mm),
        max_dt_ms=max_dt_ms, pose_distance_px=POSE_DISTANCE_PX, counts=counts,
        timing_note='Frame pairs use host receive times, which can differ from exposure times.',
        d405=dict(image_size=list(d405_size), camera_matrix=d405_matrix.tolist(),
                  factory_intrinsics=factory, stereo_dist_coeffs=d405_dist.tolist(),
                  stereo_points='undistorted_pixels' if undistort_d405 else 'raw_pixels'),
        side_view=dict(image_size=list(side_size), camera_matrix=side_matrix.tolist(),
                       dist_coeffs=side_dist.ravel().tolist(), intrinsic_views=len(side)),
        R_d405_to_side=rotation.tolist(), t_d405_to_side_mm=translation.tolist(),
        baseline_mm=float(np.linalg.norm(translation)),
        ace_pro_rms_px=side_rms, stereo_rms_px=stereo_rms,
        selected_pairs=[
            dict(d405=asdict(pair.d405_frame), side_view=asdict(pair.side_frame),
                 dt_ms=pair.delta_ms, side_order_reversed=flip,
                 d405_sharpness=pair.d405.sharpness, side_sharpness=pair.side.sharpness)
            for pair, flip in used])
    output = directory / 'stereo_calibration.json'
    with TemporaryDirectory(dir=directory) as staging:
        pending = Path(staging) / output.name
        pending.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                           encoding='utf-8')
        pending.replace(output)
    return result
