"""Batch-process the 2026-11-25 OPHIS dual-camera dataset stored on the NAS.

Supports raw .tar archives (including macOS AppleDouble ._ files) and unpacked
batch directories across single-axis (1–10 mm), dual-axis training (2–6 mm),
and dual-axis validation (3–5 mm) captures.

For each round, produces:
  1. D405 3D centerline reconstruction (depth + smooth_depth + shape3d)
  2. Ace Pro side-view 2D centerline & signed bend angle
  3. Dual-camera stereo 3D centerline triangulation via stereo_calibration.json
  4. Per-batch rounds.csv, points.csv, steady_shapes.json, hysteresis.csv
  5. Global all_rounds.csv, all_points.csv, single_axis_resolution.csv,
     all_hysteresis.csv, and summary_report.json on the NAS.

Extracts at most one .tar archive at a time into local scratch storage so SMB
random I/O and local disk usage stay bounded, and checkpoints each completed
batch so interrupted runs resume cleanly.
"""

import argparse
from bisect import bisect_right
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import sys
import tarfile
import tempfile

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from device_plugins import shape3d, vision
from pretest_data import read_capture, read_side_capture, side_angle
from single_axis_response import profiles, summarize_side_round
from stereo_calibration import DEFAULT_MAX_DT_MS, Frame, pair_frames
from stereo_reconstruction import (
    DEFAULT_MAX_REPROJ_PX,
    StereoRig,
    arc_lengths,
    deproject_d405_numpy,
    reconstruct_stereo_centerline,
)

DEFAULT_NAS_ROOT = Path('/Volumes/zhiying')
DEFAULT_OUTPUT_NAME = 'OPHIS_双机全量分析_2026-11-25'
DEFAULT_CALIBRATION_REL = Path('OPHIS_双目标定_2026-11-25') / 'stereo_calibration.json'

DATASET_GROUPS = {
    'single_axis': 'OPHIS_双机单轴位移采集_1-10mm_2026-11-25',
    'dual_axis_train': 'OPHIS_双机双轴位移采集_2-6mm_2026-11-25',
    'dual_axis_val': 'OPHIS_双机双轴验证_3-5mm_2026-11-25',
}

PIPELINE_VERSION = '1.0.0'


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_json(path, payload, *, compact=False):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    if compact:
        text = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    else:
        text = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    tmp.write_text(text, encoding='utf-8')
    tmp.replace(path)


def write_csv(path, rows, fieldnames=None):
    path = Path(path)
    if fieldnames is None:
        if not rows:
            return
        fieldnames = list(rows[0].keys())
    tmp = path.with_suffix(path.suffix + '.tmp')
    with tmp.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    tmp.replace(path)


def smooth_depth(curve, z, fx, fy):
    """Fit the distal depth trend on projected arc, retaining gaps and tip station."""
    s = arc_lengths(curve / np.array([fx, fy], dtype=np.float64))
    if len(s) == 0 or s[-1] == 0:
        return z
    s = s / s[-1]
    distal = s >= 0.4 - 1e-9
    x = (s[distal] - 0.7) / 0.3
    y = z[distal]
    valid = np.isfinite(y)
    if valid.sum() < 8:
        return z
    design = np.column_stack((np.ones_like(x), x, x * x))
    base = np.ones_like(x)
    base[-1] = 0.15
    weights = base.copy()
    for _ in range(4):
        w = np.sqrt(weights[valid])
        terms = np.linalg.lstsq(design[valid] * w[:, None], y[valid] * w, rcond=None)[0]
        fit = design @ terms
        error = np.abs(fit[valid] - y[valid])
        weights[valid] = base[valid] * np.minimum(1.0, max(0.2, 1.6 * float(np.median(error))) / np.maximum(error, 1e-6))
    t = np.clip((s[distal] - 0.4) / 0.16, 0.0, 1.0)
    result = z.copy()
    result[distal] += t * t * (3.0 - 2.0 * t) * (fit - y)
    return result


def build_d405_camera(calibration):
    """Build D405 pixel deprojector and smoothed 3D reconstructor.

    Uses pyrealsense2 when available, and falls back to exact pure-NumPy
    inverse Brown-Conrady deprojection otherwise (portable to Linux NAS/servers).
    """
    color = calibration['color']
    fx, fy = float(color['fx']), float(color['fy'])

    try:
        import pyrealsense2 as rs

        intr = rs.intrinsics()
        for key, value in color.items():
            if key not in ('model', 'model_id'):
                setattr(intr, key, value)
        intr.model = rs.distortion(color['model_id'])

        def deproject_pixel(pixel, mm):
            return (np.asarray(rs.rs2_deproject_pixel_to_point(intr, list(pixel), mm / 1000.0)) * 1000.0).tolist()
    except Exception:
        def deproject_pixel(pixel, mm):
            return deproject_d405_numpy(pixel, mm, color)

    def reconstruct(depth, orange, white, green, curve, deproject, settings):
        shaft = shape3d.orange_stations(orange, curve[0], arc_lengths(curve)[-1])

        def points(pixels, depths):
            return [deproject(p, d) if np.isfinite(d) else None for p, d in zip(pixels, depths)]

        def depths(pixels, mask):
            return np.asarray([shape3d.depth_at(depth, mask, green, p) for p in pixels], dtype=float)

        proximal = [p for p in points(shaft, depths(shaft, orange)) if p is not None]
        z = depths(curve, white)

        def fit(values):
            return shape3d.fit_shape(points(curve, values), proximal, settings['diameter_mm'], settings['length_mm'])

        original = fit(z)
        if not original['valid']:
            return original
        z[~np.isin(np.arange(len(z)), original['observed_indices'])] = np.nan
        shape = fit(smooth_depth(curve, z, fx, fy))
        if shape['valid']:
            center = np.asarray(shape['centerline_camera_mm'])
            distances = arc_lengths(original['centerline_camera_mm'])[:len(center)]
            tangent = shape3d.fit_distal_tangent(center, distances, np.asarray(shape['observed_indices']))
            if tangent is None:
                return dict(valid=False, reason='末端切线拟合失败')
            shape.update(
                tip_tangent=tangent.tolist(),
                angle_deg=shape3d.bend_angle(shape['shaft_axis']['direction'], tangent),
            )
        return shape

    return deproject_pixel, reconstruct


def round_slug(ids, target_mm, repeat):
    parts = [f'ID{axis}_{int(round(target_mm[axis - 1])):02d}mm' for axis in ids]
    return '_'.join(parts) + f'_R{int(repeat):02d}'


def detect_quality_flag(batch_name):
    lower = batch_name.lower()
    if 'tip_leak' in lower or 'leak' in lower:
        return 'tip_leak'
    return 'ok'


def safe_extract_tar(tar_path, destination):
    """Extract a .tar archive to destination, skipping macOS ._ AppleDouble files."""
    dest_resolved = Path(destination).resolve()
    extracted = 0
    try:
        with tarfile.open(tar_path, 'r:*') as archive:
            for member in archive:
                rel = Path(member.name)
                if any(part.startswith('._') or part == '.DS_Store' for part in rel.parts):
                    continue
                target = (dest_resolved / rel).resolve()
                if target != dest_resolved and dest_resolved not in target.parents:
                    raise ValueError(f'归档包含越界路径：{member.name}')
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    src = archive.extractfile(member)
                    if src is None:
                        continue
                    with src, target.open('wb') as dst:
                        shutil.copyfileobj(src, dst, length=1024 * 1024)
                    extracted += 1
    except (tarfile.TarError, EOFError, OSError) as error:
        raise RuntimeError(f'无法完整解压归档 {tar_path}（可能尚未传输完成）：{error}') from error
    return extracted


def find_batch_root(extracted_dir):
    """Locate the directory containing points.json inside an extracted tar or folder."""
    root = Path(extracted_dir)
    if (root / 'points.json').is_file():
        return root
    candidates = [p.parent for p in sorted(root.glob('*/points.json')) if not p.parent.name.startswith('.')]
    if len(candidates) == 1:
        return candidates[0]
    raise ValueError(f'未在 {root} 中找到唯一的 points.json（找到 {len(candidates)} 个）')


def discover_batches(nas_root, selected_groups=None):
    """Discover all dual-camera batch sources (.tar or unpacked folders) on the NAS."""
    nas_root = Path(nas_root)
    groups = selected_groups or list(DATASET_GROUPS.keys())
    batches = []
    for group in groups:
        folder_name = DATASET_GROUPS[group]
        group_dir = nas_root / folder_name
        if not group_dir.is_dir():
            continue
        seen_names = set()
        for entry in sorted(group_dir.iterdir()):
            if entry.name.startswith('.'):
                continue
            if entry.is_file() and entry.suffix == '.tar':
                batch_name = entry.stem
                if batch_name in seen_names:
                    continue
                seen_names.add(batch_name)
                batches.append(dict(
                    dataset_group=group,
                    group_dir=str(group_dir),
                    batch_name=batch_name,
                    source_path=str(entry),
                    source_kind='tar',
                    size_bytes=entry.stat().st_size,
                    quality_flag=detect_quality_flag(batch_name),
                ))
            elif entry.is_dir() and (entry / 'points.json').is_file():
                batch_name = entry.name
                if batch_name in seen_names:
                    continue
                seen_names.add(batch_name)
                batches.append(dict(
                    dataset_group=group,
                    group_dir=str(group_dir),
                    batch_name=batch_name,
                    source_path=str(entry),
                    source_kind='dir',
                    size_bytes=None,
                    quality_flag=detect_quality_flag(batch_name),
                ))
    return batches


def _median_or_none(values):
    clean = [float(v) for v in values if v is not None and math.isfinite(v)]
    return float(np.median(clean)) if clean else None


def _mean_or_none(values):
    clean = [float(v) for v in values if v is not None and math.isfinite(v)]
    return float(np.mean(clean)) if clean else None


def _sd_or_none(values):
    clean = [float(v) for v in values if v is not None and math.isfinite(v)]
    return float(np.std(clean, ddof=1)) if len(clean) > 1 else None


def _displacement_progress(actual_mm, ids):
    if actual_mm is None:
        return None
    vals = [actual_mm[i - 1] for i in ids]
    if any(v is None or not math.isfinite(v) for v in vals):
        return None
    if len(vals) == 1:
        return float(vals[0])
    return float(np.linalg.norm(vals))


def _compute_hysteresis_rows(rows, ids, target_mm, repeat, dataset_group, batch_name, quality_flag, modality,
                             angle_key='delta_deg'):
    """Compare advance vs return branches at matched actual displacement (0.5 mm spacing)."""
    target_norm = float(target_mm[ids[0] - 1]) if len(ids) == 1 else float(np.linalg.norm([target_mm[i - 1] for i in ids]))
    branches = []
    for phase in ('advance', 'return'):
        buckets = defaultdict(list)
        for row in rows:
            if row['phase'] != phase or not row.get('valid') or not row.get('telemetry_valid'):
                continue
            val = row.get(angle_key)
            if val is None or not math.isfinite(val):
                continue
            prog = _displacement_progress(row.get('actual_mm'), ids)
            if prog is None:
                continue
            buckets[round(prog, 3)].append(float(val))
        xs = sorted(buckets)
        branches.append((xs, [float(np.median(buckets[x])) for x in xs]))
    (out_x, out_y), (back_x, back_y) = branches
    if len(out_x) < 2 or len(back_x) < 2:
        return []
    lo = max(out_x[0], back_x[0])
    hi = min(out_x[-1], back_x[-1])
    results = []
    for mm in np.arange(0.5, target_norm, 0.5):
        if lo <= mm <= hi:
            out_val = float(np.interp(mm, out_x, out_y))
            back_val = float(np.interp(mm, back_x, back_y))
            results.append(dict(
                dataset_group=dataset_group,
                batch_name=batch_name,
                quality_flag=quality_flag,
                modality=modality,
                ids=','.join(str(i) for i in ids),
                target_q1_mm=target_mm[0],
                target_q2_mm=target_mm[1],
                target_q3_mm=target_mm[2],
                repeat=repeat,
                matched_displacement_mm=float(mm),
                advance_delta_deg=out_val,
                return_delta_deg=back_val,
                return_minus_advance_deg=back_val - out_val,
            ))
    return results


def analyze_dual_round(job):
    """Process one dual-camera round (D405 3D + Ace Pro 2D + Stereo 3D)."""
    (
        round_dir_str,
        out_dir_str,
        dataset_group,
        batch_name,
        quality_flag,
        calib_dict,
        max_dt_ms,
        max_reproj_px,
    ) = job

    cv2.setNumThreads(1)
    path = Path(round_dir_str)
    out_dir = Path(out_dir_str)

    info = read_json(path / 'round.json')
    meta = read_json(path / 'metadata.json')
    frames, events = read_capture(path)
    keys = ['reference_hold', 'T0', 'target_hold', 'return', 'T2']
    times = [events[k] for k in keys]
    if any(a >= b for a, b in zip(times, times[1:])) or len(frames) != info['frame_count'] or info['status'] != 'complete':
        raise ValueError(f'D405 采集记录不完整：{path}')

    ids = list(info['ids'])
    target_mm = list(info['target_mm'])
    reference_mm = list(info.get('reference_mm', [0, 0, 0]))
    repeat = int(info['repeat'])
    slug = round_slug(ids, target_mm, repeat)
    cycle_end = times[-1] - times[1]

    # ---- 1. D405 stream ----
    deproject_pixel, reconstruct = build_d405_camera(meta['calibration'])
    depth_scale_mm = float(meta['calibration']['depth_scale_m']) * 1000.0
    d405_rows = []
    d405_frame_objs = []
    for frame in frames:
        stamp = float(frame['host_receive_monotonic'])
        phase = ('before', 'reference', 'advance', 'hold', 'return', 'after')[bisect_right(times, stamp)]
        with np.load(path / frame['file'], allow_pickle=False) as raw:
            result, _ = vision.analyze(
                raw['color_bgr'],
                raw['aligned_depth'] * depth_scale_mm,
                meta['settings'],
                deproject=deproject_pixel,
                reconstruction=reconstruct,
            )
        shape = result['shape_3d']
        axes = sorted(frame['telemetry']['axes'], key=lambda a: a['id'])
        d405_rows.append(dict(
            file=frame['file'],
            frame_number=frame.get('frame_number'),
            host_receive_monotonic=stamp,
            t_from_T0_s=stamp - times[1],
            phase=phase,
            actual_mm=[a['position'] for a in axes],
            telemetry_valid=bool(frame['telemetry']['valid']),
            curve_valid=bool(result['valid']),
            curve_px=result.get('curve_px', []),
            projected_theta_deg=result.get('projected_theta_deg'),
            depth_fraction=result.get('depth_fraction'),
            valid=bool(shape['valid']),
            reason=shape['reason'],
            angle_deg=shape.get('angle_deg'),
            centerline_camera_mm=shape.get('centerline_camera_mm', []),
            tip_tangent=shape.get('tip_tangent'),
            shaft_direction=(shape.get('shaft_axis') or {}).get('direction'),
            observed_indices=shape.get('observed_indices', []),
        ))
        d405_frame_objs.append(Frame(frame['file'], stamp, frame.get('frame_number')))

    d405_ref = [r for r in d405_rows if r['phase'] == 'reference' and r['valid']]
    d405_hold = [r for r in d405_rows if r['phase'] == 'hold' and r['valid']]
    d405_baseline = _median_or_none([r['angle_deg'] for r in d405_ref])
    for r in d405_rows:
        r['delta_deg'] = (r['angle_deg'] - d405_baseline) if (r['valid'] and d405_baseline is not None) else None

    d405_out = out_dir / 'd405_frames' / f'{slug}.jsonl'
    with d405_out.open('w', encoding='utf-8') as stream:
        for r in d405_rows:
            stream.write(json.dumps(r, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')

    # ---- 2. Side-view (Ace Pro) stream ----
    has_side = bool(info.get('side_view')) and (path / 'side_view' / 'frames.jsonl').is_file()
    side_rows = []
    side_frame_objs = []
    side_summary = None
    if has_side:
        side_dir = path / 'side_view'
        side_meta = read_json(side_dir / 'metadata.json')
        side_frames = read_side_capture(path)
        if len(side_frames) != info['side_view']['frame_count']:
            raise ValueError(f'侧视采集记录帧数不一致：{path}')
        size = side_meta['format']
        prev_stamp = float('-inf')
        for frame in side_frames:
            stamp = float(frame['host_receive_monotonic'])
            if not math.isfinite(stamp) or stamp <= prev_stamp:
                raise ValueError(f'侧视帧时间无效：{path}')
            prev_stamp = stamp
            phase = ('before', 'reference', 'advance', 'hold', 'return', 'after')[bisect_right(times, stamp)]
            image_path = (side_dir / frame['file']).resolve()
            if image_path.parent != side_dir.resolve():
                raise ValueError(f'侧视帧路径无效：{image_path}')
            raw_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if raw_bgr is None or raw_bgr.shape[:2] != (size['height'], size['width']):
                raise ValueError(f'无法读取侧视原始帧：{image_path}')
            res, _ = vision.analyze_rgb(raw_bgr, side_meta['settings'])
            reading = side_angle(res)
            telemetry = frame.get('telemetry') or {}
            feedback = {item['id']: item for item in telemetry.get('axes', [])}
            actual, offsets = [], []
            for axis_id in (1, 2, 3):
                sample = feedback.get(axis_id, {})
                offset = sample['last_seen'] - stamp if sample.get('last_seen', 0) > 0 else None
                offsets.append(offset)
                actual.append(
                    sample['position']
                    if telemetry.get('valid') and offset is not None and abs(offset) <= 1.0
                    and sample.get('online', True) and not sample.get('error')
                    else None
                )
            side_rows.append(dict(
                file=frame['file'],
                frame_number=frame.get('frame_number'),
                host_receive_monotonic=stamp,
                t_from_T0_s=stamp - times[1],
                phase=phase,
                actual_mm=actual,
                feedback_offset_s=offsets,
                telemetry_valid=all(v is not None for v in actual),
                reference_position=all(
                    v is not None and abs(v - q) <= 0.05 for v, q in zip(actual, reference_mm)
                ),
                valid=reading is not None,
                reason=res['reason'],
                angle_deg=reading,
                curve_px=res['curve_px'],
            ))
            side_frame_objs.append(Frame(frame['file'], stamp, frame.get('frame_number')))

        side_summary = summarize_side_round(side_rows, cycle_end=cycle_end)
        side_out = out_dir / 'side_frames' / f'{slug}.jsonl'
        with side_out.open('w', encoding='utf-8') as stream:
            for r in side_rows:
                stream.write(json.dumps(r, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')

    # ---- 3. Dual-camera stereo 3D reconstruction ----
    stereo_rows = []
    if has_side and calib_dict is not None:
        rig = StereoRig.from_dict(calib_dict)
        d405_by_file = {r['file']: r for r in d405_rows}
        side_by_file = {r['file']: r for r in side_rows}
        matched_pairs = pair_frames(d405_frame_objs, side_frame_objs, max_dt_ms=max_dt_ms)
        length_mm = float(meta['settings'].get('length_mm', 30.0))

        for d_frame, s_frame in matched_pairs:
            d_row = d405_by_file[d_frame.file]
            s_row = side_by_file[s_frame.file]
            dt_ms = abs(d_frame.host_receive_monotonic - s_frame.host_receive_monotonic) * 1000.0
            stereo = reconstruct_stereo_centerline(
                rig,
                d_row['curve_px'] if d_row['curve_valid'] else [],
                s_row['curve_px'] if s_row['valid'] else [],
                length_mm=length_mm,
                max_reproj_px=max_reproj_px,
                shaft_direction=d_row['shaft_direction'],
            )
            angle_diff = None
            tip_diff_mm = None
            if stereo['valid'] and d_row['valid']:
                angle_diff = float(stereo['angle_deg'] - d_row['angle_deg'])
                if len(d_row['centerline_camera_mm']) == 21 and len(stereo['centerline_camera_mm']) == 21:
                    tip_diff_mm = float(np.linalg.norm(
                        np.asarray(stereo['centerline_camera_mm'][-1]) - np.asarray(d_row['centerline_camera_mm'][-1])
                    ))
            stereo_rows.append(dict(
                d405_file=d_frame.file,
                side_file=s_frame.file,
                dt_ms=dt_ms,
                host_receive_monotonic=d_frame.host_receive_monotonic,
                t_from_T0_s=d_row['t_from_T0_s'],
                phase=d_row['phase'],
                actual_mm=d_row['actual_mm'],
                telemetry_valid=d_row['telemetry_valid'],
                d405_vs_stereo_angle_diff_deg=angle_diff,
                d405_vs_stereo_tip_mm=tip_diff_mm,
                **stereo,
            ))

        stereo_ref = [r for r in stereo_rows if r['phase'] == 'reference' and r['valid']]
        stereo_baseline = _median_or_none([r['angle_deg'] for r in stereo_ref])
        for r in stereo_rows:
            r['delta_deg'] = (r['angle_deg'] - stereo_baseline) if (r['valid'] and stereo_baseline is not None) else None

        stereo_out = out_dir / 'stereo_frames' / f'{slug}.jsonl'
        with stereo_out.open('w', encoding='utf-8') as stream:
            for r in stereo_rows:
                stream.write(json.dumps(r, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
    else:
        stereo_ref = []
        stereo_baseline = None

    # ---- 4. Round-level summary metrics ----
    def _tip_metrics(ref_records, hold_records):
        if not ref_records or not hold_records:
            return None, [None, None, None]
        ref_tan = np.median([r['tip_tangent'] for r in ref_records], axis=0)
        hold_tan = np.median([r['tip_tangent'] for r in hold_records], axis=0)
        turn = float(shape3d.bend_angle(ref_tan, hold_tan))
        ref_tips = [r['centerline_camera_mm'][-1] for r in ref_records if len(r['centerline_camera_mm']) == 21]
        hold_tips = [r['centerline_camera_mm'][-1] for r in hold_records if len(r['centerline_camera_mm']) == 21]
        delta = (np.median(hold_tips, axis=0) - np.median(ref_tips, axis=0)).tolist() if (ref_tips and hold_tips) else [None, None, None]
        return turn, delta

    d405_turn, d405_tip_delta = _tip_metrics(d405_ref, d405_hold)
    d405_cycle = [r for r in d405_rows if r['valid'] and 0.0 <= r['t_from_T0_s'] <= cycle_end]
    d405_peak = max(d405_cycle, key=lambda r: r['angle_deg'], default=None)

    stereo_hold = [r for r in stereo_rows if r['phase'] == 'hold' and r['valid']]
    stereo_turn, stereo_tip_delta = _tip_metrics(stereo_ref, stereo_hold)
    stereo_cycle = [r for r in stereo_rows if r['valid'] and 0.0 <= r['t_from_T0_s'] <= cycle_end]
    stereo_peak = max(stereo_cycle, key=lambda r: r['angle_deg'], default=None)

    actual_hold = [
        _median_or_none([r['actual_mm'][axis_idx] for r in d405_rows if r['phase'] == 'hold' and r['telemetry_valid']])
        for axis_idx in range(3)
    ]

    round_row = dict(
        dataset_group=dataset_group,
        batch_name=batch_name,
        quality_flag=quality_flag,
        ids=','.join(str(i) for i in ids),
        target_q1_mm=target_mm[0],
        target_q2_mm=target_mm[1],
        target_q3_mm=target_mm[2],
        repeat=repeat,
        round_slug=slug,
        round_dir=path.name,
        actual_hold_q1_mm=actual_hold[0],
        actual_hold_q2_mm=actual_hold[1],
        actual_hold_q3_mm=actual_hold[2],
        # D405 3D
        d405_frames=len(d405_rows),
        d405_valid=sum(r['valid'] for r in d405_rows),
        d405_ref_valid=len(d405_ref),
        d405_hold_valid=len(d405_hold),
        d405_ref_deg=d405_baseline,
        d405_hold_deg=_median_or_none([r['angle_deg'] for r in d405_hold]),
        d405_hold_delta_deg=_median_or_none([r['delta_deg'] for r in d405_hold]),
        d405_ref_sd_deg=_sd_or_none([r['angle_deg'] for r in d405_ref]),
        d405_hold_sd_deg=_sd_or_none([r['angle_deg'] for r in d405_hold]),
        d405_tip_turn_deg=d405_turn,
        d405_tip_dx_mm=d405_tip_delta[0],
        d405_tip_dy_mm=d405_tip_delta[1],
        d405_tip_dz_mm=d405_tip_delta[2],
        d405_peak_delta_deg=d405_peak['delta_deg'] if d405_peak else None,
        # Side-view 2D
        side_frames=side_summary['frames'] if side_summary else 0,
        side_ref_valid=side_summary['reference_valid'] if side_summary else 0,
        side_hold_valid=side_summary['hold_valid'] if side_summary else 0,
        side_ref_deg=side_summary['reference_deg'] if side_summary else None,
        side_hold_delta_deg=side_summary['hold_delta_deg'] if side_summary else None,
        side_ref_noise_p95_deg=side_summary['reference_noise_abs_p95_deg'] if side_summary else None,
        side_peak_delta_deg=side_summary['peak_delta_deg'] if side_summary else None,
        side_return_residual_deg=side_summary['return_residual_deg'] if side_summary else None,
        # Stereo 3D
        stereo_paired_frames=len(stereo_rows),
        stereo_valid=sum(r['valid'] for r in stereo_rows),
        stereo_ref_valid=len(stereo_ref),
        stereo_hold_valid=len(stereo_hold),
        stereo_ref_deg=stereo_baseline,
        stereo_hold_deg=_median_or_none([r['angle_deg'] for r in stereo_hold]),
        stereo_hold_delta_deg=_median_or_none([r['delta_deg'] for r in stereo_hold]),
        stereo_ref_sd_deg=_sd_or_none([r['angle_deg'] for r in stereo_ref]),
        stereo_hold_sd_deg=_sd_or_none([r['angle_deg'] for r in stereo_hold]),
        stereo_tip_turn_deg=stereo_turn,
        stereo_tip_dx_mm=stereo_tip_delta[0],
        stereo_tip_dy_mm=stereo_tip_delta[1],
        stereo_tip_dz_mm=stereo_tip_delta[2],
        stereo_peak_delta_deg=stereo_peak['delta_deg'] if stereo_peak else None,
        stereo_length_hold_mm=_median_or_none([r['length_mm'] for r in stereo_hold]),
        stereo_reproj_rms_px=_median_or_none([r['reproj_rms_px'] for r in stereo_rows if r['valid']]),
        d405_vs_stereo_angle_diff_deg=_median_or_none(
            [r['d405_vs_stereo_angle_diff_deg'] for r in stereo_hold if r['d405_vs_stereo_angle_diff_deg'] is not None]
        ),
        d405_vs_stereo_tip_mm=_median_or_none(
            [r['d405_vs_stereo_tip_mm'] for r in stereo_hold if r['d405_vs_stereo_tip_mm'] is not None]
        ),
    )

    # ---- 5. Steady shapes & hysteresis ----
    d405_ref_curve = profiles(d405_ref)
    d405_hold_curve = profiles(d405_hold)
    stereo_ref_curve = profiles(stereo_ref)
    stereo_hold_curve = profiles(stereo_hold)

    steady_shape = dict(
        dataset_group=dataset_group,
        batch_name=batch_name,
        quality_flag=quality_flag,
        ids=ids,
        target_mm=target_mm,
        repeat=repeat,
        round_slug=slug,
        d405_reference_camera_mm=d405_ref_curve.tolist() if d405_ref_curve is not None else None,
        d405_hold_camera_mm=d405_hold_curve.tolist() if d405_hold_curve is not None else None,
        d405_root_relative_change_mm=(
            ((d405_hold_curve - d405_hold_curve[0]) - (d405_ref_curve - d405_ref_curve[0])).tolist()
            if d405_ref_curve is not None and d405_hold_curve is not None
            else None
        ),
        stereo_reference_camera_mm=stereo_ref_curve.tolist() if stereo_ref_curve is not None else None,
        stereo_hold_camera_mm=stereo_hold_curve.tolist() if stereo_hold_curve is not None else None,
        stereo_root_relative_change_mm=(
            ((stereo_hold_curve - stereo_hold_curve[0]) - (stereo_ref_curve - stereo_ref_curve[0])).tolist()
            if stereo_ref_curve is not None and stereo_hold_curve is not None
            else None
        ),
    )

    hysteresis = []
    hysteresis.extend(_compute_hysteresis_rows(
        d405_rows, ids, target_mm, repeat, dataset_group, batch_name, quality_flag, 'd405'
    ))
    if side_rows:
        hysteresis.extend(_compute_hysteresis_rows(
            side_rows, ids, target_mm, repeat, dataset_group, batch_name, quality_flag, 'side_view'
        ))
    if stereo_rows:
        hysteresis.extend(_compute_hysteresis_rows(
            stereo_rows, ids, target_mm, repeat, dataset_group, batch_name, quality_flag, 'stereo'
        ))

    # Reference noise p95 for resolution table
    d405_ref_p95 = (
        float(np.percentile([abs(r['angle_deg'] - d405_baseline) for r in d405_ref], 95))
        if d405_ref and d405_baseline is not None
        else None
    )
    stereo_ref_p95 = (
        float(np.percentile([abs(r['angle_deg'] - stereo_baseline) for r in stereo_ref], 95))
        if stereo_ref and stereo_baseline is not None
        else None
    )
    round_row['d405_ref_noise_p95_deg'] = d405_ref_p95
    round_row['stereo_ref_noise_p95_deg'] = stereo_ref_p95

    return round_row, steady_shape, hysteresis


def summarize_points(round_rows):
    grouped = defaultdict(list)
    for row in round_rows:
        key = (
            row['dataset_group'],
            row['quality_flag'],
            row['ids'],
            int(row['target_q1_mm']),
            int(row['target_q2_mm']),
            int(row['target_q3_mm']),
        )
        grouped[key].append(row)

    points = []
    for (group, qflag, ids_str, q1, q2, q3), items in sorted(grouped.items()):
        def stats(field):
            vals = [float(r[field]) for r in items if r.get(field) not in (None, '') and math.isfinite(float(r[field]))]
            return (
                len(vals),
                float(np.mean(vals)) if vals else None,
                float(np.std(vals, ddof=1)) if len(vals) > 1 else None,
                min(vals, default=None),
                max(vals, default=None),
            )

        d_n, d_mean, d_sd, d_min, d_max = stats('d405_hold_delta_deg')
        s_n, s_mean, s_sd, s_min, s_max = stats('side_hold_delta_deg')
        st_n, st_mean, st_sd, st_min, st_max = stats('stereo_hold_delta_deg')
        st_turn_n, st_turn_mean, st_turn_sd, _, _ = stats('stereo_tip_turn_deg')
        _, reproj_mean, _, _, _ = stats('stereo_reproj_rms_px')
        points.append(dict(
            dataset_group=group,
            quality_flag=qflag,
            ids=ids_str,
            target_q1_mm=q1,
            target_q2_mm=q2,
            target_q3_mm=q3,
            recorded_repeats=len(items),
            d405_valid_repeats=d_n,
            d405_mean_delta_deg=d_mean,
            d405_repeat_sd_deg=d_sd,
            d405_min_delta_deg=d_min,
            d405_max_delta_deg=d_max,
            side_valid_repeats=s_n,
            side_mean_delta_deg=s_mean,
            side_repeat_sd_deg=s_sd,
            side_min_delta_deg=s_min,
            side_max_delta_deg=s_max,
            stereo_valid_repeats=st_n,
            stereo_mean_delta_deg=st_mean,
            stereo_repeat_sd_deg=st_sd,
            stereo_min_delta_deg=st_min,
            stereo_max_delta_deg=st_max,
            stereo_tip_turn_valid_repeats=st_turn_n,
            stereo_tip_turn_mean_deg=st_turn_mean,
            stereo_tip_turn_sd_deg=st_turn_sd,
            stereo_reproj_rms_mean_px=reproj_mean,
        ))
    return points


def build_single_axis_resolution(all_rounds):
    """Compute per-axis 1–10 mm resolution and adjacent-level separability for D405 and Stereo."""
    rows = []
    for modality, delta_key, noise_key in (
        ('d405', 'd405_hold_delta_deg', 'd405_ref_noise_p95_deg'),
        ('stereo', 'stereo_hold_delta_deg', 'stereo_ref_noise_p95_deg'),
        ('side_view', 'side_hold_delta_deg', 'side_ref_noise_p95_deg'),
    ):
        levels = defaultdict(list)
        noises = defaultdict(list)
        for r in all_rounds:
            if r['dataset_group'] != 'single_axis' or r['quality_flag'] != 'ok':
                continue
            ids = [int(x) for x in str(r['ids']).split(',')]
            if len(ids) != 1:
                continue
            axis = ids[0]
            mm = int(r[f'target_q{axis}_mm'])
            val = r.get(delta_key)
            if val not in (None, '') and math.isfinite(float(val)):
                levels[axis, mm].append(float(val))
            nval = r.get(noise_key)
            if nval not in (None, '') and math.isfinite(float(nval)):
                noises[axis, mm].append(float(nval))
        for axis in (1, 2, 3):
            for mm in range(1, 11):
                cur = levels[axis, mm]
                prev = levels[axis, mm - 1]
                threshold = max(noises[axis, mm], default=None)
                outside = (
                    (min(cur) > threshold or max(cur) < -threshold)
                    if len(cur) >= 3 and threshold is not None
                    else None
                )
                disjoint = (
                    (min(cur) > max(prev) or min(prev) > max(cur))
                    if len(cur) >= 3 and len(prev) >= 3
                    else None
                )
                rows.append(dict(
                    modality=modality,
                    axis=axis,
                    target_mm=mm,
                    valid_repeats=len(cur),
                    reference_abs_p95_max_deg=threshold,
                    all_repeats_outside_reference_noise_same_sign=outside,
                    adjacent_repeat_ranges_disjoint=disjoint,
                ))
    return rows


def process_batch(batch_spec, output_root, calib_dict, *, workers, scratch_dir, max_dt_ms, max_reproj_px, force):
    """Extract (if .tar), process all rounds in parallel, and atomically publish batch outputs."""
    batch_dest = Path(output_root) / 'batches' / batch_spec['dataset_group'] / batch_spec['batch_name']
    meta_file = batch_dest / 'batch_meta.json'
    if meta_file.is_file() and not force:
        print(f"[跳过已完成] {batch_spec['dataset_group']} / {batch_spec['batch_name']}", flush=True)
        return read_json(meta_file)

    batch_dest.parent.mkdir(parents=True, exist_ok=True)
    scratch_parent = Path(scratch_dir) if scratch_dir else None
    if scratch_parent is not None:
        scratch_parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix=f"ophis_src_{batch_spec['batch_name']}_", dir=scratch_parent) as src_tmp:
        if batch_spec['source_kind'] == 'tar':
            print(f"[解压] {batch_spec['source_path']} -> {src_tmp}", flush=True)
            extracted = safe_extract_tar(batch_spec['source_path'], src_tmp)
            print(f"       已解压 {extracted} 个文件", flush=True)
            batch_root = find_batch_root(src_tmp)
        else:
            batch_root = Path(batch_spec['source_path'])

        manifest = read_json(batch_root / 'points.json')
        completed = manifest.get('completed_rounds', [])
        if not completed:
            raise ValueError(f"批次中无已完成轮次：{batch_spec['source_path']}")

        round_paths = []
        for entry in completed:
            rpath = (batch_root / entry['path']).resolve(strict=True)
            if rpath.parent != batch_root.resolve():
                raise ValueError(f'轮次路径越界：{rpath}')
            round_paths.append(rpath)

        # Stage results locally first, then publish to NAS to minimize SMB random writes.
        with tempfile.TemporaryDirectory(prefix=f"ophis_out_{batch_spec['batch_name']}_", dir=scratch_parent) as stage_tmp:
            stage = Path(stage_tmp) / 'batch'
            (stage / 'd405_frames').mkdir(parents=True)
            (stage / 'side_frames').mkdir(parents=True)
            (stage / 'stereo_frames').mkdir(parents=True)

            jobs = [
                (
                    str(rpath),
                    str(stage),
                    batch_spec['dataset_group'],
                    batch_spec['batch_name'],
                    batch_spec['quality_flag'],
                    calib_dict,
                    max_dt_ms,
                    max_reproj_px,
                )
                for rpath in round_paths
            ]

            round_rows, steady_shapes, hysteresis_rows = [], [], []
            total = len(jobs)
            if workers == 1:
                results = map(analyze_dual_round, jobs)
                for idx, (rrow, sshape, hrows) in enumerate(results, 1):
                    round_rows.append(rrow)
                    steady_shapes.append(sshape)
                    hysteresis_rows.extend(hrows)
                    print(
                        f"  [{batch_spec['batch_name']}] {idx}/{total} {rrow['round_slug']} "
                        f"D405={rrow['d405_hold_valid']}/{rrow['d405_frames']} "
                        f"Side={rrow['side_hold_valid']}/{rrow['side_frames']} "
                        f"Stereo={rrow['stereo_hold_valid']}/{rrow['stereo_paired_frames']}",
                        flush=True,
                    )
            else:
                with ProcessPoolExecutor(max_workers=workers) as pool:
                    for idx, (rrow, sshape, hrows) in enumerate(pool.map(analyze_dual_round, jobs), 1):
                        round_rows.append(rrow)
                        steady_shapes.append(sshape)
                        hysteresis_rows.extend(hrows)
                        print(
                            f"  [{batch_spec['batch_name']}] {idx}/{total} {rrow['round_slug']} "
                            f"D405={rrow['d405_hold_valid']}/{rrow['d405_frames']} "
                            f"Side={rrow['side_hold_valid']}/{rrow['side_frames']} "
                            f"Stereo={rrow['stereo_hold_valid']}/{rrow['stereo_paired_frames']}",
                            flush=True,
                        )

            round_rows.sort(key=lambda r: (r['target_q1_mm'], r['target_q2_mm'], r['target_q3_mm'], r['repeat']))
            steady_shapes.sort(key=lambda s: (*s['target_mm'], s['repeat']))
            point_rows = summarize_points(round_rows)

            write_csv(stage / 'rounds.csv', round_rows)
            write_csv(stage / 'points.csv', point_rows)
            write_json(stage / 'steady_shapes.json', steady_shapes, compact=True)
            if hysteresis_rows:
                write_csv(stage / 'hysteresis.csv', hysteresis_rows)
            else:
                write_csv(stage / 'hysteresis.csv', [], fieldnames=[
                    'dataset_group', 'batch_name', 'quality_flag', 'modality', 'ids',
                    'target_q1_mm', 'target_q2_mm', 'target_q3_mm', 'repeat',
                    'matched_displacement_mm', 'advance_delta_deg', 'return_delta_deg',
                    'return_minus_advance_deg',
                ])

            batch_meta = dict(
                pipeline_version=PIPELINE_VERSION,
                completed_at=datetime.now().astimezone().isoformat(),
                dataset_group=batch_spec['dataset_group'],
                batch_name=batch_spec['batch_name'],
                quality_flag=batch_spec['quality_flag'],
                source_path=batch_spec['source_path'],
                source_kind=batch_spec['source_kind'],
                ids=manifest.get('ids'),
                points_mm=manifest.get('points_mm'),
                repeats=manifest.get('repeats'),
                round_count=len(round_rows),
                stereo_enabled=calib_dict is not None,
            )
            write_json(stage / 'batch_meta.json', batch_meta)

            # Atomic publish onto NAS: copy to sibling hidden dir, then replace.
            nas_staging = batch_dest.parent / f".staging_{batch_spec['batch_name']}"
            if nas_staging.exists():
                shutil.rmtree(nas_staging)
            shutil.copytree(stage, nas_staging)
            if batch_dest.exists():
                shutil.rmtree(batch_dest)
            nas_staging.rename(batch_dest)
            print(f"[完成] {batch_dest}", flush=True)
            return batch_meta


def aggregate_outputs(output_root, calibration_path=None):
    """Aggregate all completed batches under output_root/batches/ into top-level tables."""
    output_root = Path(output_root)
    batches_dir = output_root / 'batches'
    if not batches_dir.is_dir():
        raise ValueError(f'未找到已完成批次目录：{batches_dir}')

    all_rounds = []
    all_hysteresis = []
    batch_metas = []

    for meta_path in sorted(batches_dir.glob('*/*/batch_meta.json')):
        bdir = meta_path.parent
        batch_metas.append(read_json(meta_path))
        with (bdir / 'rounds.csv').open(encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                all_rounds.append(row)
        hyst_file = bdir / 'hysteresis.csv'
        if hyst_file.is_file():
            with hyst_file.open(encoding='utf-8') as stream:
                for row in csv.DictReader(stream):
                    all_hysteresis.append(row)

    if not all_rounds:
        raise ValueError(f'没有可汇总的轮次数据：{batches_dir}')

    # Normalize numeric keys for sorting & point aggregation.
    def _sort_key(r):
        return (
            r['dataset_group'],
            r['quality_flag'],
            r['ids'],
            int(r['target_q1_mm']),
            int(r['target_q2_mm']),
            int(r['target_q3_mm']),
            int(r['repeat']),
        )

    all_rounds.sort(key=_sort_key)
    all_points = summarize_points(all_rounds)
    resolution_rows = build_single_axis_resolution(all_rounds)

    write_csv(output_root / 'all_rounds.csv', all_rounds)
    write_csv(output_root / 'all_points.csv', all_points)
    write_csv(output_root / 'single_axis_resolution.csv', resolution_rows)
    if all_hysteresis:
        write_csv(output_root / 'all_hysteresis.csv', all_hysteresis)

    # Summary statistics by dataset_group and quality_flag
    group_stats = {}
    for group in sorted({r['dataset_group'] for r in all_rounds}):
        for qflag in sorted({r['quality_flag'] for r in all_rounds if r['dataset_group'] == group}):
            subset = [r for r in all_rounds if r['dataset_group'] == group and r['quality_flag'] == qflag]
            reproj = [float(r['stereo_reproj_rms_px']) for r in subset if r.get('stereo_reproj_rms_px') not in (None, '')]
            ang_diff = [
                abs(float(r['d405_vs_stereo_angle_diff_deg']))
                for r in subset
                if r.get('d405_vs_stereo_angle_diff_deg') not in (None, '')
            ]
            tip_diff = [
                float(r['d405_vs_stereo_tip_mm'])
                for r in subset
                if r.get('d405_vs_stereo_tip_mm') not in (None, '')
            ]
            group_stats[f'{group}:{qflag}'] = dict(
                rounds=len(subset),
                d405_hold_valid_rounds=sum(int(r['d405_hold_valid']) > 0 for r in subset),
                side_hold_valid_rounds=sum(int(r['side_hold_valid']) > 0 for r in subset),
                stereo_hold_valid_rounds=sum(int(r['stereo_hold_valid']) > 0 for r in subset),
                stereo_reproj_rms_median_px=_median_or_none(reproj),
                d405_vs_stereo_abs_angle_median_deg=_median_or_none(ang_diff),
                d405_vs_stereo_tip_median_mm=_median_or_none(tip_diff),
            )

    report = dict(
        pipeline_version=PIPELINE_VERSION,
        generated_at=datetime.now().astimezone().isoformat(),
        calibration_path=str(calibration_path) if calibration_path else None,
        completed_batches=len(batch_metas),
        total_rounds=len(all_rounds),
        clean_rounds=sum(r['quality_flag'] == 'ok' for r in all_rounds),
        flagged_rounds=sum(r['quality_flag'] != 'ok' for r in all_rounds),
        groups=group_stats,
        batches=batch_metas,
    )
    write_json(output_root / 'summary_report.json', report)
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nas-root', type=Path, default=DEFAULT_NAS_ROOT,
                        help=f'NAS 根目录（默认 {DEFAULT_NAS_ROOT}）')
    parser.add_argument('--output', type=Path, default=None,
                        help=f'结果保存目录（默认 <nas-root>/{DEFAULT_OUTPUT_NAME}）')
    parser.add_argument('--calibration', type=Path, default=None,
                        help=f'双目标定 JSON 路径（默认 <nas-root>/{DEFAULT_CALIBRATION_REL}）')
    parser.add_argument('--no-stereo', action='store_true',
                        help='跳过双目立体三维重建，仅运行 D405 与侧视分析')
    parser.add_argument('--datasets', nargs='+', choices=list(DATASET_GROUPS.keys()), default=None,
                        help='仅处理指定数据集分组（默认全部三组）')
    parser.add_argument('--workers', type=int, default=4, help='单批次内并行进程数（默认 4）')
    parser.add_argument('--scratch-dir', type=Path, default=None,
                        help='本地临时解压与暂存目录（默认系统临时目录，每次仅解压一个 .tar）')
    parser.add_argument('--max-dt-ms', type=float, default=DEFAULT_MAX_DT_MS,
                        help=f'双机时间配对最大时间差 ms（默认 {DEFAULT_MAX_DT_MS}）')
    parser.add_argument('--max-reproj-px', type=float, default=DEFAULT_MAX_REPROJ_PX,
                        help=f'双目三角化最大重投影误差 px（默认 {DEFAULT_MAX_REPROJ_PX}）')
    parser.add_argument('--force', action='store_true', help='忽略已有断点，重新处理全部批次')
    parser.add_argument('--aggregate-only', action='store_true', help='仅合并已完成批次生成总表')
    parser.add_argument('--dry-run', action='store_true', help='仅扫描并打印待处理批次计划，不执行处理')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.workers < 1:
        raise SystemExit('--workers 须大于等于 1')

    nas_root = args.nas_root
    output_root = args.output if args.output is not None else (nas_root / DEFAULT_OUTPUT_NAME)
    calib_path = args.calibration if args.calibration is not None else (nas_root / DEFAULT_CALIBRATION_REL)

    if args.aggregate_only:
        report = aggregate_outputs(output_root, calibration_path=calib_path if calib_path.is_file() else None)
        print(f"已汇总 {report['completed_batches']} 个批次、共 {report['total_rounds']} 轮 -> {output_root}", flush=True)
        return 0

    batches = discover_batches(nas_root, args.datasets)
    if not batches:
        print(f'未在 {nas_root} 下发现待处理的双机数据集目录。', file=sys.stderr)
        return 1

    calib_dict = None
    if not args.no_stereo:
        if calib_path.is_file():
            calib_dict = read_json(calib_path)
        elif args.dry_run:
            print(f'[警告] 未找到双目标定文件：{calib_path}（上传完成后可自动读取）', flush=True)
        else:
            print(f'错误：未找到双目标定文件 {calib_path}（如暂不做双目重建可加 --no-stereo）', file=sys.stderr)
            return 1

    print(f'NAS 根目录：{nas_root}', flush=True)
    print(f'结果输出目录：{output_root}', flush=True)
    print(f'双目标定文件：{calib_path if calib_dict is not None else "未启用"}', flush=True)
    print(f'发现 {len(batches)} 个批次：', flush=True)
    for b in batches:
        size_gb = f"{b['size_bytes'] / 1e9:.2f} GB" if b['size_bytes'] is not None else 'dir'
        done = (output_root / 'batches' / b['dataset_group'] / b['batch_name'] / 'batch_meta.json').is_file()
        status = '已完成(跳过)' if (done and not args.force) else '待处理'
        flag = f" [标记:{b['quality_flag']}]" if b['quality_flag'] != 'ok' else ''
        print(f"  - {b['dataset_group']:15s} | {b['batch_name']:28s} | {size_gb:9s} | {status}{flag}", flush=True)

    if args.dry_run:
        return 0

    output_root.mkdir(parents=True, exist_ok=True)
    for b in batches:
        process_batch(
            b,
            output_root,
            calib_dict,
            workers=args.workers,
            scratch_dir=args.scratch_dir,
            max_dt_ms=args.max_dt_ms,
            max_reproj_px=args.max_reproj_px,
            force=args.force,
        )

    report = aggregate_outputs(output_root, calibration_path=calib_path if calib_dict is not None else None)
    print(
        f"全部完成：{report['completed_batches']} 个批次，共 {report['total_rounds']} 轮"
        f"（正常 {report['clean_rounds']} 轮，异常标记 {report['flagged_rounds']} 轮）-> {output_root}",
        flush=True,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
