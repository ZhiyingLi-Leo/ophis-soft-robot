"""Summarize reconstructed single-axis rounds in camera coordinates."""
import csv
import json
from collections import defaultdict

import numpy as np

from pretest_data import signed_delta


def profiles(rows):
    """Put each valid frame on 21 equal 3D arc-length stations before aggregation."""
    curves = []
    for row in rows:
        curve = np.asarray(row['centerline_camera_mm'])
        distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))]
        stations = np.linspace(0, distance[-1], 21)
        curves.append(np.column_stack([np.interp(stations, distance, axis) for axis in curve.T]))
    return np.median(curves, axis=0) if curves else None


def summarize_response(folder):
    noise = defaultdict(list)
    levels = defaultdict(list)
    loops = []
    shapes = []
    for path in sorted((folder/'frames').glob('*.jsonl')):
        axis, target, repeat = path.stem.split('_')
        key = int(axis[2:]), int(target[:-2])
        with path.open() as stream:
            rows = [r for line in stream if (r := json.loads(line))['valid']]
        reference = [r for r in rows if r['phase']=='reference']
        hold = [r for r in rows if r['phase']=='hold']
        if not reference or not hold:
            continue
        ref = np.median([r['angle_deg'] for r in reference])
        noise[key].append(float(np.percentile([abs(r['angle_deg']-ref) for r in reference], 95)))
        levels[key].append(float(np.median([r['angle_deg'] for r in hold])-ref))
        a, b = profiles(reference), profiles(hold)
        shapes.append(dict(axis=key[0],target_mm=key[1],repeat=int(repeat[1:]),
                           reference_camera_mm=a.tolist(), hold_camera_mm=b.tolist(),
                           root_relative_change_mm=((b-b[0])-(a-a[0])).tolist()))
        # Compare branches only within their overlapping measured position range.
        branches = []
        for phase in ('advance','return'):
            samples = defaultdict(list)
            for r in rows:
                if r['phase']==phase and r['telemetry_valid']:
                    samples[r['actual_mm'][key[0]-1]].append(r['angle_deg']-ref)
            x = sorted(samples)
            branches.append((x,[np.median(samples[v]) for v in x]))
        (out_x,out_y),(back_x,back_y) = branches
        if len(out_x)<2 or len(back_x)<2:
            continue
        for mm in np.arange(.5,key[1],.5):
            if max(out_x[0],back_x[0]) <= mm <= min(out_x[-1],back_x[-1]):
                out = np.interp(mm,out_x,out_y)
                back = np.interp(mm,back_x,back_y)
                loops.append((key[0],key[1],int(repeat[1:]),mm,out,back,back-out))
    with (folder/'steady_shapes.json').open('x') as f:
        json.dump(shapes, f, allow_nan=False, separators=(',',':'))
    with (folder/'hysteresis.csv').open('x',newline='') as f:
        w = csv.writer(f)
        w.writerow(['axis','target_mm','repeat','matched_position_mm','advance_delta_deg','return_delta_deg','return_minus_advance_deg'])
        w.writerows(loops)
    with (folder/'resolution.csv').open('x',newline='') as f:
        w = csv.writer(f)
        w.writerow(['axis','target_mm','valid_repeats','reference_abs_p95_max_deg',
                    'all_repeats_outside_reference_noise_same_sign','adjacent_repeat_ranges_disjoint'])
        for axis in range(1,4):
            for mm in range(1,11):
                key = axis,mm
                delta = levels[key]
                threshold = max(noise[key],default=None)
                prev = levels[axis,mm-1]
                outside = (min(delta)>threshold or max(delta)<-threshold) if len(delta)==3 else None
                disjoint = (min(delta)>max(prev) or min(prev)>max(delta)) if len(prev)==len(delta)==3 else None
                w.writerow((axis,mm,len(delta),threshold,outside,disjoint))

def summarize_side_round(rows, *, cycle_end):
    reference = [row for row in rows if row['phase'] == 'reference' and row['valid']]
    held = [row for row in rows if row['phase'] == 'hold' and row['valid']]
    baseline = None
    if reference:
        first = reference[0]['angle_deg']
        center = first + np.median([signed_delta(row['angle_deg'], first) for row in reference])
        baseline = float(signed_delta(center, 0))
    for row in rows:
        row['delta_deg'] = signed_delta(row['angle_deg'], baseline) if row['valid'] and baseline is not None else None
    ref_errors = [abs(row['delta_deg']) for row in reference]
    hold_delta = float(np.median([row['delta_deg'] for row in held])) if held and baseline is not None else None
    cycle = [row for row in rows if 0 <= row['t_from_T0_s'] <= cycle_end and row['delta_deg'] is not None]
    peak = max(cycle, key=lambda row: abs(row['delta_deg']), default=None)
    # Use the same final arrival window as the encoder stability check.
    residual = [row['delta_deg'] for row in cycle if row['reference_position'] and
                row['phase'] == 'return' and row['t_from_T0_s'] >= cycle_end-.6]
    result = dict(frames=len(rows), reference_deg=baseline, hold_delta_deg=hold_delta,
                  reference_noise_abs_p95_deg=float(np.percentile(ref_errors, 95)) if ref_errors else None,
                  peak_file=peak['file'] if peak else None,
                  peak_t_from_T0_s=peak['t_from_T0_s'] if peak else None,
                  peak_delta_deg=peak['delta_deg'] if peak else None,
                  return_residual_deg=float(np.median(residual)) if residual else None)
    for phase in ('reference', 'advance', 'hold', 'return'):
        phase_rows = [row for row in rows if row['phase'] == phase]
        result[phase+'_frames'] = len(phase_rows)
        result[phase+'_valid'] = sum(row['valid'] for row in phase_rows)
    return result


def summarize_side_points(rounds):
    grouped = defaultdict(list)
    for row in rounds:
        grouped[row['axis'], row['target_mm']].append(row)
    points = []
    for (axis, target), group in sorted(grouped.items()):
        values = [row['hold_delta_deg'] for row in group if row['hold_delta_deg'] is not None]
        points.append(dict(axis=axis, target_mm=target, recorded_repeats=len(group), valid_repeats=len(values),
                           mean_delta_deg=float(np.mean(values)) if values else None,
                           repeat_sd_deg=float(np.std(values, ddof=1)) if len(values)>1 else None,
                           min_delta_deg=min(values, default=None), max_delta_deg=max(values, default=None)))
    return points
