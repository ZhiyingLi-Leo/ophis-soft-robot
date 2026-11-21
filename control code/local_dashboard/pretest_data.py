"""Summarize and export one D405 round from the original, timestamped frames."""

import json
from contextlib import ExitStack
from statistics import median


def read_capture(path):
    with (path / 'frames.jsonl').open() as index:
        items = [json.loads(line) for line in index]
    frames = [item for item in items if item['kind'] == 'frame']
    events = {item['note']['kind']: item['host_monotonic'] for item in items
              if item['kind'] == 'annotation'}
    return frames, events


def save_json(path, result):
    pending = path.with_suffix('.json.tmp')
    pending.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    pending.replace(path)


def relative_angle(angle, zero):
    return None if angle is None else (angle - zero + 180) % 360 - 180


def frame_angle(frame):
    return frame['measurement'].get('projected_signed_deg')


def summarize(frames, events, zero, noise):
    stamps = [frame['host_receive_monotonic'] for frame in frames]
    angles = [relative_angle(frame_angle(frame), zero) for frame in frames]
    cycle = [i for i, stamp in enumerate(stamps) if events['T0'] <= stamp <= events['T2']]
    if len(cycle) < 3:
        raise RuntimeError('T0–T2 内的记录帧不足')
    start, end = cycle[0], cycle[-1]

    def trigger(i, stamp, angle=None):
        frame = frames[i]
        axes = sorted(frame['telemetry'].get('axes', []), key=lambda a: a['id'])
        return dict(frame=frame['frame_number'], file=frame['file'], host_monotonic=stamp,
                    frame_monotonic=stamps[i], relative_angle_deg=angles[i] if angle is None else angle,
                    feedback=[dict(id=a['id'], mm=a['position'], host_monotonic=a['last_seen']) for a in axes])

    # A three-frame median suppresses isolated contour jumps; never bridge missing detections.
    candidates = []
    for i in range(start + 1, end):
        window = angles[i-1:i+2]
        if None not in window:
            value = median(window)
            candidates.append((abs(value), -abs(angles[i] - value), i, value))
    peak = max(candidates, key=lambda item: (item[0], item[1]), default=None)
    peak_trigger = (dict(status='detected', **trigger(peak[2], stamps[peak[2]], peak[3]))
                    if peak and peak[0] > noise else dict(status='indistinguishable', frame=None))
    hold = [angle for stamp, angle in zip(stamps, angles)
            if events['target_hold'] <= stamp < events['return'] and angle is not None]
    triggers = dict(T0=trigger(start, events['T0']), T1=peak_trigger, T2=trigger(end, events['T2']))
    return triggers, angles, (sum(hold) / len(hold) if hold else None)


def video_writer(path, codec, fps, size, *, depth=False):
    import cv2
    options = [cv2.VIDEOWRITER_PROP_IS_COLOR, int(not depth)]
    if depth:
        options += [cv2.VIDEOWRITER_PROP_DEPTH, cv2.CV_16U]
    writer = cv2.VideoWriter(str(path), cv2.CAP_FFMPEG, cv2.VideoWriter_fourcc(*codec), fps, size, options)
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(f'无法打开视频编码器 {codec}（{path.name}）')
    return writer


def check_video_support(directory, width, height):
    import tempfile
    from pathlib import Path
    with tempfile.TemporaryDirectory(dir=directory) as scratch, ExitStack() as stack:
        for name, codec, depth in [('color.mkv', 'FFV1', False), ('depth.mkv', 'FFV1', True),
                                   ('annotated.mp4', 'mp4v', False)]:
            stack.callback(video_writer(Path(scratch) / name, codec, 5, (width, height), depth=depth).release)


def export_videos(path, frames, angles, triggers):
    import cv2
    import numpy as np

    with np.load(path / frames[0]['file']) as first:
        height, width = first['color_bgr'].shape[:2]
    first_time = frames[0]['host_receive_monotonic']
    elapsed = frames[-1]['host_receive_monotonic'] - first_time
    fps = (len(frames) - 1) / elapsed if elapsed > 0 else 5
    markers = {name: value.get('file') for name, value in triggers.items()}
    tracks = [('color_raw.mkv', 'FFV1', False), ('depth_raw.mkv', 'FFV1', True),
              ('angle_overlay.mp4', 'mp4v', False)]
    with ExitStack() as stack:
        writers = []
        for name, codec, depth in tracks:
            temporary = path / ('partial_' + name)
            writer = video_writer(temporary, codec, fps, (width, height), depth=depth)
            stack.callback(writer.release)
            writers.append(writer)
        for frame, angle in zip(frames, angles):
            with np.load(path / frame['file']) as arrays:
                color = arrays['color_bgr']
                writers[0].write(color)
                writers[1].write(arrays['native_depth'])
            curve = np.rint(frame['measurement'].get('curve_px', [])).astype(np.int32)
            if len(curve) >= 2:
                cv2.circle(color, tuple(curve[0]), 4, (255, 100, 0), -1)
                cv2.circle(color, tuple(curve[-1]), 4, (0, 0, 255), -1)
            stamp = frame['host_receive_monotonic'] - triggers['T0']['host_monotonic']
            reading = '---' if angle is None else f'{angle:+.2f} deg'
            labels = ' '.join(name for name, file in markers.items() if file == frame['file'])
            caption = f'2D projection: {reading}   {stamp:+.2f}s   {labels}'
            cv2.rectangle(color, (0, 0), (width, 34), (21, 26, 32), -1)
            cv2.putText(color, caption, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 255, 255), 1, cv2.LINE_AA)
            writers[2].write(color)
    for name, _, _ in tracks:
        (path / ('partial_' + name)).replace(path / name)
    return dict(fps=fps, files=[name for name, _, _ in tracks])
