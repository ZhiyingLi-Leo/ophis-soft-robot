"""Summarize reconstructed single-axis rounds in camera coordinates."""
import csv
import json
from collections import defaultdict

import numpy as np


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
