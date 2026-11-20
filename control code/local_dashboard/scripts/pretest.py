"""D405 pre-test with parallel cylinder targets and per-round recordings."""

import argparse
import math
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from statistics import mean, median, stdev

from experiment_api import Robot
from pretest_data import check_video_support, export_videos, frame_angle, read_capture, relative_angle, save_json, summarize
from sampling import axes_by_id, require_camera

POINTS = ((2, 0, 0), (1, 0, 0), (0, 2, 0), (0, 0, 2),
          (2, 1, 0), (0, 2, 1), (1, 0, 2), (1, 1, 1), (2, 1, 1))
REPEATS = 3
POSITION_TOLERANCE = .05
HOLD_SECONDS = 2


class Pretest:
    def __init__(self, robot, limit):
        self.robot = robot
        self.limit = limit
        self.group = 'pretest_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.last_valid = time.monotonic()
        self.completed = 0
        self.segment = 1

    def progress(self, phase, *, begin_round=False, **info):
        return self.robot.command('pretest_progress', begin_round=begin_round,
                                  progress=dict(phase=phase, completed=self.completed,
                                                total=len(POINTS)*REPEATS, **info))['result']

    def feedback(self, *, recording=False, armed=False):
        state = self.robot.state()
        camera = require_camera(state, recording=recording)
        axes = axes_by_id(state)
        if any(not a['online'] or a['error'] or a['age'] is None or a['age'] > 1
               or (armed and not a['armed']) for a in axes):
            raise RuntimeError('三轴反馈异常或已停止')
        if camera['measurement_settings'] != self.settings:
            raise RuntimeError('相机识别参数已变化')
        angle = (camera.get('measurement') or {}).get('projected_signed_deg')
        now = time.monotonic()
        if angle is not None:
            self.last_valid = now
        elif now - self.last_valid > 1:
            raise RuntimeError('柔性段轮廓持续丢失')
        return axes, camera, angle

    def at(self, axes, target):
        return all(not a['moving'] and abs(a['position'] - q) <= POSITION_TOLERANCE
                   for a, q in zip(axes, target))

    def arm(self):
        if not self.at(self.feedback()[0], self.reference):
            raise RuntimeError('三轴已偏离本次参考位，请检查装配后重新确认')
        for axis in (1, 2, 3):
            self.robot.arm(axis)

    def stop(self):
        errors = []
        for axis in (1, 2, 3):
            try:
                self.robot.stop_cylinder(axis)
            except RuntimeError as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('部分电缸停止请求失败：' + '；'.join(errors))

    def record(self, label):
        return Path(self.robot.device('camera.d405', 'record_start', label=label, group=self.group)['session_path'])

    def baseline(self):
        self.progress('baseline')
        path = self.record('reference')
        try:
            calibration = self.robot.camera_state()['calibration']['color']
            check_video_support(path, calibration['width'], calibration['height'])
            end = time.monotonic() + 30
            while time.monotonic() < end:
                if not self.at(self.feedback(recording=True)[0], self.reference):
                    raise RuntimeError('采集参考姿态时电缸发生位移')
                self.robot.wait(.1)
        finally:
            self.robot.record_stop()
        frames, _ = read_capture(path)
        angles = [angle for frame in frames if (angle := frame_angle(frame)) is not None]
        if len(angles) < 5:
            raise RuntimeError('参考姿态的有效角度帧不足')
        self.zero = angles[0] + median(relative_angle(angle, angles[0]) for angle in angles)
        deviations = sorted(abs(relative_angle(angle, self.zero)) for angle in angles)
        self.noise = max(.5, 2*deviations[math.ceil(.95*len(deviations)) - 1])
        save_json(path / 'reference.json', dict(reference_mm=self.reference, zero_deg=self.zero,
                                               peak_threshold_deg=self.noise, valid_frames=len(angles)))
        self.progress('ready', reference_mm=self.reference, zero_deg=self.zero,
                      noise_deg=self.noise, directory=str(path.parent))
        return path.parent

    def observe(self, target):
        axes, camera, angle = self.feedback(recording=True, armed=True)
        if (any(not lo-POSITION_TOLERANCE <= a['position'] <= hi+POSITION_TOLERANCE
                for a, lo, hi in zip(axes, self.reference, target))
                or sum(a['position'] - lo for a, lo in zip(axes, self.reference)) > self.limit+3*POSITION_TOLERANCE):
            raise RuntimeError('实测位置超出本轮并行路径的位移范围')
        return axes, camera, angle

    def wait_arrival(self, destination, target):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if self.at(self.observe(target)[0], destination):
                return
            self.robot.wait(.08)
        raise RuntimeError('电缸到位超时')

    def wait_reference(self, target):
        deadline = time.monotonic() + 30
        window = deque()
        previous = None
        while time.monotonic() < deadline:
            axes, camera, angle = self.observe(target)
            if not self.at(axes, self.reference) or angle is None:
                window.clear()
            elif camera['frame_number'] != previous:
                previous = camera['frame_number']
                stamp = camera['last_frame_time']
                window.append((stamp, relative_angle(angle, self.zero)))
                while len(window) > 2 and window[1][0] <= stamp - HOLD_SECONDS:
                    window.popleft()
                values = [v for _, v in window]
                if len(values) >= 5 and stamp-window[0][0] >= HOLD_SECONDS and max(values)-min(values) <= 2*self.noise:
                    return
            self.robot.wait(.08)
        raise RuntimeError('返回参考位后未在规定时间内稳定')

    def wait_preroll(self, target):
        deadline = time.monotonic() + 4
        first_frame_seen = None
        while time.monotonic() < deadline:
            axes, camera, angle = self.observe(target)
            if not self.at(axes, self.reference):
                raise RuntimeError('起始参考位发生偏移')
            if camera['recorded_frames'] and angle is not None:
                now = time.monotonic()
                if first_frame_seen is None:
                    first_frame_seen = now
                elif camera['recorded_frames'] >= 2 and now - first_frame_seen >= .6:
                    return
            self.robot.wait(.06)
        raise RuntimeError('起始参考姿态的有效画面不足')

    def capture_round(self, point, repeat, offset, target):
        path = self.record(f'point_{point:02d}_repeat_{repeat}')
        result = dict(status='incomplete', point=point, repeat=repeat, segment=self.segment, reference_mm=self.reference,
                      offset_mm=offset, target_mm=target, metric='signed_2d_projection',
                      zero_deg=self.zero, peak_threshold_deg=self.noise)
        try:
            save_json(path / 'round.json', result)
            self.wait_preroll(target)
            self.robot.annotate(kind='T0', point=point, repeat=repeat, target=target)
            self.robot.move_all(target)
            self.progress('advance')
            self.wait_arrival(target, target)
            self.robot.annotate(kind='target_hold')
            self.progress('hold')
            end = time.monotonic() + HOLD_SECONDS
            while time.monotonic() < end:
                if not self.at(self.observe(target)[0], target):
                    raise RuntimeError('目标保持期间电缸位置发生偏移')
                self.robot.wait(.08)
            self.robot.annotate(kind='return')
            self.robot.move_all(self.reference)
            self.progress('return')
            self.wait_reference(target)
            self.robot.annotate(kind='T2')
        except Exception:
            self.stop()
            raise
        finally:
            self.robot.record_stop()

        self.progress('export')
        frames, events = read_capture(path)
        triggers, angles, holding_angle = summarize(frames, events, self.zero, self.noise)
        result.update(status='captured', triggers=triggers, target_hold_mean_deg=holding_angle,
                      duration_s=events['T2'] - events['T0'], frame_count=len(frames))
        save_json(path / 'round.json', result)
        result.update(status='complete', videos=export_videos(path, frames, angles, triggers))
        save_json(path / 'round.json', result)
        self.completed += 1
        peak = triggers['T1'].get('relative_angle_deg')
        self.progress('round_complete', last_peak_deg=peak, last_round=str(path))
        print(f'点 {point} · 第 {repeat} 轮 · {result["duration_s"]:.1f}s · 峰值 {peak if peak is not None else "不可分辨"} · {path}', flush=True)
        return dict(path=path.name, segment=self.segment, peak_deg=peak, residual_deg=triggers['T2']['relative_angle_deg'])

    def begin(self, point, repeat, offset):
        while not self.progress('prepare', begin_round=True, point=point, repeat=repeat, offset_mm=offset):
            self.stop()
            self.progress('paused')
            while self.robot.state()['experiment']['pause_requested']:
                self.robot.wait(.2)
            self.arm()
            self.segment += 1

    def run(self):
        camera = require_camera(self.robot.state())
        self.settings = camera['settings']
        axes, _, angle = self.feedback()
        if angle is None or any(a['moving'] or not math.isclose(a['speed'], .5) for a in axes):
            raise RuntimeError('Pre-test 需要有效轮廓、静止电缸和各轴 0.5 mm/s 的速度配置')
        self.reference = [max(a['minimum'], a['position']) for a in axes]
        targets = [[q+x for q, x in zip(self.reference, offset)] for offset in POINTS]
        for offset, target in zip(POINTS, targets):
            if sum(offset) > self.limit or any(not a['minimum'] <= q <= a['maximum'] for a, q in zip(axes, target)):
                raise RuntimeError(f'Pre-test 点 {offset} 超出本次位移限制，请检查参考位、行程和组合上限')
        directory = self.baseline()
        groups = []
        try:
            self.arm()
            for point, (offset, target) in enumerate(zip(POINTS, targets), 1):
                group = dict(offset_mm=offset, rounds=[])
                groups.append(group)
                for repeat in range(1, REPEATS+1):
                    self.begin(point, repeat, offset)
                    group['rounds'].append(self.capture_round(point, repeat, offset, target))
                    peaks = [abs(r['peak_deg']) for r in group['rounds'] if r['peak_deg'] is not None]
                    group.update(detected=len(peaks), mean_peak_deg=mean(peaks) if peaks else None,
                                 sample_sd_deg=stdev(peaks) if len(peaks) == REPEATS else None)
                    save_json(directory / 'points.json', groups)
            self.progress('done')
        finally:
            self.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-advance-mm', required=True, type=float)
    args = parser.parse_args()
    if not math.isfinite(args.max_advance_mm) or args.max_advance_mm <= 0:
        parser.error('三轴推进增量之和上限须为正数')
    with Robot() as robot:
        Pretest(robot, args.max_advance_mm).run()
