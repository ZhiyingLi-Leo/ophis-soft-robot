"""Reconstruct the formal single-axis D405 dataset (IDs 1–3, 1–10 mm, three repeats).

The input directory contains one points.json per batch. Supply a new sibling
directory for output. frames/*.jsonl contains one record per original frame,
including invalid estimates, stage, actual encoder positions and camera-space
geometry in mm. The distal tangent is evaluated at 85% of the original arc.
rounds.csv uses per-window medians and the difference from the same round's
pre-T0 median. points.csv summarizes independent rounds. steady_shapes.json
stores camera-space median curves on 21 equal normalized arc-length stations.
resolution.csv compares the three repeats against pre-motion angular noise
and against the adjacent level's observed range. hysteresis.csv interpolates
the two motion branches at matched actual positions (0.5 mm spacing).
"""
import argparse
from bisect import bisect_right
import csv
import json
import tempfile
import sys
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from device_plugins import shape3d, vision
from pretest_data import read_capture, read_side_capture, side_angle
from single_axis_response import summarize_response, summarize_side_points, summarize_side_round


def arc(points):
    return np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def smooth_depth(curve, z, intr):
    """Fit the distal depth trend on projected arc, retaining gaps and tip station."""
    s = arc(curve / [intr.fx, intr.fy])
    if s[-1] == 0:
        return z
    s /= s[-1]
    distal = s >= .4 - 1e-9
    x = (s[distal] - .7) / .3
    y = z[distal]
    valid = np.isfinite(y)
    if valid.sum() < 8:
        return z
    design = np.column_stack((np.ones_like(x), x, x*x))
    base = np.ones_like(x)
    base[-1] = .15
    weights = base.copy()
    for _ in range(4):
        w = np.sqrt(weights[valid])
        terms = np.linalg.lstsq(design[valid]*w[:, None], y[valid]*w, rcond=None)[0]
        fit = design @ terms
        error = abs(fit[valid] - y[valid])
        weights[valid] = base[valid] * np.minimum(1, max(.2, 1.6*np.median(error))/np.maximum(error, 1e-6))
    t = np.clip((s[distal]-.4)/.16, 0, 1)
    result = z.copy()
    result[distal] += t*t*(3-2*t)*(fit-y)
    return result


def camera(calibration):
    import pyrealsense2 as rs

    intr = rs.intrinsics()
    for key, value in calibration['color'].items():
        if key not in ('model', 'model_id'):
            setattr(intr, key, value)
    intr.model = rs.distortion(calibration['color']['model_id'])

    def deproject_pixel(pixel, mm):
        return (np.asarray(rs.rs2_deproject_pixel_to_point(intr, list(pixel), mm/1000))*1000).tolist()

    def reconstruct(depth, orange, white, green, curve, deproject, settings):
        shaft = shape3d.orange_stations(orange, curve[0], arc(curve)[-1])
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
        shape = fit(smooth_depth(curve, z, intr))
        if shape['valid']:
            tangent = shape3d.fit_distal_tangent(np.asarray(shape['centerline_camera_mm']),
                                                arc(original['centerline_camera_mm']), np.asarray(shape['observed_indices']))
            if tangent is None:
                return dict(valid=False, reason='末端切线拟合失败')
            shape.update(tip_tangent=tangent.tolist(), angle_deg=shape3d.bend_angle(shape['shaft_axis']['direction'], tangent))
        return shape
    return deproject_pixel, reconstruct


def read_json(path):
    return json.loads(path.read_text())


def write_csv(path, rows):
    with path.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def analyze_round(job):
    path, output = job
    cv2.setNumThreads(1)
    info = read_json(path/'round.json')
    meta = read_json(path/'metadata.json')
    deproject_pixel, reconstruct = camera(meta['calibration'])
    frames, events = read_capture(path)
    keys = ['reference_hold', 'T0', 'target_hold', 'return', 'T2']
    times = [events[k] for k in keys]
    if times != sorted(times) or len(frames) != info['frame_count'] or info['status'] != 'complete':
        raise ValueError(f'采集记录不完整 {path}')
    axis, = info['ids']
    target = info['target_mm'][axis-1]
    name = f"ID{axis}_{target:02d}mm_R{info['repeat']:02d}"
    rows = []
    for frame in frames:
        stamp = frame['host_receive_monotonic']
        phase = ('before', 'reference', 'advance', 'hold', 'return', 'after')[np.searchsorted(times, stamp, side='right')]
        with np.load(path/frame['file'], allow_pickle=False) as raw:
            result, _ = vision.analyze(raw['color_bgr'], raw['aligned_depth']*(meta['calibration']['depth_scale_m']*1000),
                                       meta['settings'], deproject=deproject_pixel, reconstruction=reconstruct)
        shape = result['shape_3d']
        axes = sorted(frame['telemetry']['axes'], key=lambda a: a['id'])
        rows.append(dict(file=frame['file'], t_from_T0_s=stamp-times[1], phase=phase,
                         actual_mm=[a['position'] for a in axes], telemetry_valid=frame['telemetry']['valid'],
                         valid=shape['valid'], reason=shape['reason'], angle_deg=shape.get('angle_deg'),
                         centerline_camera_mm=shape.get('centerline_camera_mm', []), tip_tangent=shape.get('tip_tangent'),
                         shaft_direction=(shape.get('shaft_axis') or {}).get('direction'),
                         observed_indices=shape.get('observed_indices', [])))
    def window(phase):
        return [r for r in rows if r['phase'] == phase and r['valid']]
    ref, hold = window('reference'), window('hold')
    def mid(records, field):
        return np.median([r[field] for r in records], axis=0)
    baseline = float(mid(ref, 'angle_deg')) if ref else None
    for r in rows:
        r['delta_deg'] = r['angle_deg']-baseline if r['valid'] and ref else None
    out = output/(name+'.jsonl')
    with out.open('x') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n')
    cycle = [r for r in rows if r['valid'] and 0 <= r['t_from_T0_s'] <= times[-1]-times[1]]
    peak = max(cycle, key=lambda r:r['angle_deg'], default=None)
    actual = [r['actual_mm'][axis-1] for r in rows if r['phase']=='hold' and r['telemetry_valid']]
    tip_delta = [None]*3
    if ref and hold:
        tip_delta = (mid(hold,'centerline_camera_mm')[-1]-mid(ref,'centerline_camera_mm')[-1]).tolist()
    return dict(axis=axis, target_mm=target, repeat=info['repeat'], source=str(path), frames=len(rows),
                reference_valid=len(ref), hold_valid=len(hold), valid=sum(r['valid'] for r in rows),
                reference_deg=baseline, hold_deg=float(mid(hold,'angle_deg')) if hold else None,
                hold_delta_deg=float(mid(hold,'delta_deg')) if ref and hold else None,
                reference_sd_deg=np.std([r['angle_deg'] for r in ref],ddof=1) if len(ref)>1 else None,
                hold_sd_deg=np.std([r['angle_deg'] for r in hold],ddof=1) if len(hold)>1 else None,
                actual_hold_mm=np.median(actual) if actual else None,
                tip_turn_deg=shape3d.bend_angle(mid(ref,'tip_tangent'),mid(hold,'tip_tangent')) if ref and hold else None,
                tip_dx_mm=tip_delta[0],tip_dy_mm=tip_delta[1],tip_dz_mm=tip_delta[2],
                peak_file=peak['file'] if peak else None, peak_delta_deg=peak['delta_deg'] if peak else None)


def analyze_side_round(job):
    path, output = job
    cv2.setNumThreads(1)
    info = read_json(path/'round.json')
    metadata = read_json(path/'side_view'/'metadata.json')
    overhead, events = read_capture(path)
    frames = read_side_capture(path)
    keys = ['reference_hold', 'T0', 'target_hold', 'return', 'T2']
    times = [events[key] for key in keys]
    if (info['status'] != 'complete' or len(overhead) != info['frame_count'] or
            len(frames) != info['side_view']['frame_count'] or
            any(a >= b for a,b in zip(times, times[1:]))):
        raise ValueError(f'采集记录不完整 {path}')
    axis, = info['ids']
    target = info['target_mm'][axis-1]
    name = f"ID{axis}_{target:02d}mm_R{info['repeat']:02d}"
    rows = []
    previous = float('-inf')
    for frame in frames:
        stamp = frame['host_receive_monotonic']
        if not np.isfinite(stamp) or stamp <= previous:
            raise ValueError(f'侧视帧时间无效 {path}')
        previous = stamp
        phase = ('before','reference','advance','hold','return','after')[bisect_right(times,stamp)]
        directory = path/'side_view'
        image = (directory/frame['file']).resolve()
        if image.parent != directory.resolve():
            raise ValueError(f'侧视帧路径无效 {image}')
        raw = cv2.imread(str(image), cv2.IMREAD_COLOR)
        size = metadata['format']
        if raw is None or raw.shape[:2] != (size['height'], size['width']):
            raise ValueError(f'无法读取侧视原始帧 {image}')
        result, _ = vision.analyze_rgb(raw, metadata['settings'])
        reading = side_angle(result)
        telemetry = frame.get('telemetry') or {}
        feedback = {item['id']:item for item in telemetry.get('axes', [])}
        actual, offsets = [], []
        for axis_id in (1,2,3):
            sample = feedback.get(axis_id, {})
            offset = sample['last_seen'] - stamp if sample.get('last_seen',0)>0 else None
            offsets.append(offset)
            actual.append(sample['position'] if telemetry.get('valid') and offset is not None and
                          abs(offset)<=1 and sample.get('online',True) and not sample.get('error') else None)
        rows.append(dict(file=frame['file'], frame_number=frame['frame_number'], host_receive_monotonic=stamp,
                         t_from_T0_s=stamp-times[1], phase=phase, actual_mm=actual, feedback_offset_s=offsets,
                         telemetry_valid=all(value is not None for value in actual),
                         reference_position=all(value is not None and abs(value-q)<=.05
                                                for value,q in zip(actual, info['reference_mm'])),
                         valid=reading is not None, reason=result['reason'],
                         angle_deg=reading, centerline_px=result['curve_px']))
    summary = summarize_side_round(rows, cycle_end=times[-1]-times[1])
    hold_positions = [row['actual_mm'][axis-1] for row in rows
                      if row['phase']=='hold' and row['actual_mm'][axis-1] is not None]
    with (output/(name+'.jsonl')).open('x',encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    return dict(axis=axis,target_mm=target,repeat=info['repeat'],source=str(path),
                actual_hold_mm=float(np.median(hold_positions)) if hold_positions else None, **summary)


def analyze_side_batches(source, destination, workers):
    rounds = []
    seen = set()
    manifests = [source/'points.json'] if (source/'points.json').is_file() else sorted(source.glob('*/points.json'))
    for manifest in manifests:
        for entry in read_json(manifest)['completed_rounds']:
            path = (manifest.parent/entry['path']).resolve(strict=True)
            if path.parent != manifest.parent.resolve():
                raise ValueError(path)
            info = read_json(path/'round.json')
            if not info.get('side_view') or len(info['ids']) != 1:
                continue
            axis, = info['ids']
            key = (axis, info['target_mm'][axis-1], info['repeat'])
            if key in seen:
                raise ValueError(f'重复工况 {key}')
            seen.add(key)
            rounds.append(path)
    if not rounds:
        raise ValueError('未找到含侧视记录的已完成单轴采集轮次')
    with tempfile.TemporaryDirectory(prefix='.single_analysis_', dir=destination.parent) as tmp:
        stage = Path(tmp)/'results'
        side = stage/'side_view'
        details = side/'frames'
        details.mkdir(parents=True)
        jobs = [(path,details) for path in rounds]
        if workers == 1:
            summaries = list(map(analyze_side_round, jobs))
        else:
            with ProcessPoolExecutor(workers) as pool:
                summaries = list(pool.map(analyze_side_round, jobs))
        summaries.sort(key=lambda row:(row['axis'],row['target_mm'],row['repeat']))
        write_csv(side/'rounds.csv', summaries)
        write_csv(side/'points.csv', summarize_side_points(summaries))
        stage.rename(destination)
    print(destination,flush=True)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('source', type=Path)
    cli.add_argument('output', type=Path)
    cli.add_argument('--workers', type=int, default=4)
    cli.add_argument('--side-view', action='store_true', help='分析已完成的侧视 RGB 单轴轮次')
    args = cli.parse_args()
    src, dst = args.source.resolve(strict=True), args.output.resolve()
    if dst.exists() or src == dst or src in dst.parents:
        cli.error('请选择原始数据目录之外的新结果目录')
    if args.workers < 1:
        cli.error('--workers 须大于等于 1')
    if args.side_view:
        analyze_side_batches(src, dst, args.workers)
        return
    rounds = []
    expected = {(i,m,r) for i in range(1,4) for m in range(1,11) for r in range(1,4)}
    seen = set()
    for manifest in sorted(src.glob('*/points.json')):
        for entry in read_json(manifest)['completed_rounds']:
            p = (manifest.parent/entry['path']).resolve(strict=True)
            if p.parent != manifest.parent:
                raise ValueError(p)
            info = read_json(p/'round.json')
            axis, = info['ids']
            key = (axis, info['target_mm'][axis-1], info['repeat'])
            if key in seen:
                raise ValueError(f'重复工况 {key}')
            seen.add(key)
            rounds.append(p)
    if seen != expected:
        raise ValueError('需要 ID1–3、1–10 mm、每点三轮的完整数据')
    # Publish only after every worker finishes and its frame records can be read back.
    with tempfile.TemporaryDirectory(prefix='.single_analysis_', dir=dst.parent) as tmp:
        stage = Path(tmp)/'results'
        details = stage/'frames'
        details.mkdir(parents=True)
        with ProcessPoolExecutor(args.workers) as pool:
            summaries = []
            for row in pool.map(analyze_round, [(p,details) for p in rounds]):
                summaries.append(row)
                print(f"{len(summaries)}/90  ID{row['axis']} {row['target_mm']} mm R{row['repeat']}",flush=True)
        grouped = defaultdict(list)
        for row in summaries:
            grouped[row['axis'],row['target_mm']].append(row)
        points = []
        for (axis,mm), group in sorted(grouped.items()):
            delta = [r['hold_delta_deg'] for r in group if r['hold_delta_deg'] is not None]
            points.append(dict(axis=axis,target_mm=mm,valid_repeats=len(delta),
                               mean_delta_deg=np.mean(delta) if delta else None,
                               repeat_sd_deg=np.std(delta,ddof=1) if len(delta)>1 else None,
                               min_delta_deg=min(delta,default=None),max_delta_deg=max(delta,default=None)))
        write_csv(stage/'rounds.csv', summaries)
        write_csv(stage/'points.csv', points)
        for row in summaries:
            path = details/f"ID{row['axis']}_{row['target_mm']:02d}mm_R{row['repeat']:02d}.jsonl"
            with path.open() as f:
                if sum(1 for line in f if isinstance(json.loads(line)['valid'],bool)) != row['frames']:
                    raise IOError(path)
        for file, count in [('rounds.csv',90),('points.csv',30)]:
            with (stage/file).open() as f:
                if len(list(csv.DictReader(f))) != count:
                    raise IOError(file)
        summarize_response(stage)
        stage.rename(dst)
        print(dst,flush=True)


if __name__ == '__main__':
    main()
