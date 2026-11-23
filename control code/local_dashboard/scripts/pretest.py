"""D405 pre-test with parallel cylinder targets and per-round recordings."""

import argparse
import math
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from statistics import mean, median, stdev

from experiment_api import Robot
from pretest_data import check_video_support, export_videos, frame_angle, read_capture, save_json, shape_angle, summarize
from sampling import ArrivalWindow, axes_by_id, positions_at_target, require_camera

POINTS = ((2, 0, 0), (1, 0, 0), (0, 2, 0), (0, 0, 2),
          (2, 1, 0), (0, 2, 1), (1, 0, 2), (1, 1, 1), (2, 1, 1))
REPEATS = 3
POSITION_TOLERANCE = .05
HOLD_SECONDS = 2


def direction_difference(left, right):
    dot = sum(a*b for a,b in zip(left,right))
    return math.degrees(math.acos(max(-1,min(1,dot))))


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
        angle = shape_angle(camera.get('measurement'))
        now = time.monotonic()
        if angle is not None:
            self.last_valid = now
        elif now - self.last_valid > 1:
            reason = ((camera.get('measurement') or {}).get('shape_3d') or {}).get('reason', '')
            raise RuntimeError('有效三维角持续丢失。' + reason)
        return axes, camera, angle

    def at(self, axes, target):
        return positions_at_target(axes,target,POSITION_TOLERANCE)

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
        valid = [(angle,frame['measurement']['shape_3d']['tip_tangent'])
                 for frame in frames if (angle := frame_angle(frame)) is not None]
        angles = [angle for angle, _ in valid]
        if len(angles) < 5:
            raise RuntimeError('参考姿态的有效三维角帧不足')
        self.reference_angle = median(angles)
        deviations = sorted(abs(angle-self.reference_angle) for angle in angles)
        quantile = math.ceil(.95*len(valid))-1
        self.noise = max(.5, 2*deviations[quantile])
        directions = [tip for _,tip in valid]
        middle = [median(component) for component in zip(*directions)]
        norm = math.hypot(*middle)
        if norm < 1e-6:
            raise RuntimeError('静止参考姿态的尖端方向无法确定')
        middle = [component/norm for component in middle]
        spread = sorted(direction_difference(tip,middle) for tip in directions)
        self.direction_noise = max(.5, 2*spread[quantile])
        save_json(path / 'reference.json', dict(reference_mm=self.reference, angle_3d_deg=self.reference_angle,
                                               peak_threshold_deg=self.noise, tip_direction_noise_deg=self.direction_noise,
                                               valid_frames=len(angles)))
        self.progress('ready', reference_mm=self.reference, reference_angle_3d_deg=self.reference_angle,
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
        measured = ArrivalWindow(destination,POSITION_TOLERANCE)
        while time.monotonic() < deadline:
            if measured.update(self.observe(target)[0]):
                return
            self.robot.wait(.08)
        raise RuntimeError('电缸到位超时')

    def wait_reference(self, target):
        deadline = time.monotonic() + 30
        window = deque()
        previous = None
        measured = ArrivalWindow(self.reference,POSITION_TOLERANCE)
        while time.monotonic() < deadline:
            axes, camera, angle = self.observe(target)
            if not measured.update(axes) or angle is None:
                window.clear()
            elif camera['frame_number'] != previous:
                previous = camera['frame_number']
                stamp = camera['last_frame_time']
                tip = camera['measurement']['shape_3d']['tip_tangent']
                window.append((stamp, angle, tip))
                while len(window) > 2 and window[1][0] <= stamp - HOLD_SECONDS:
                    window.popleft()
                values = [v for _, v, _ in window]
                direction_stable = all(direction_difference(direction,window[0][2]) <= 2*self.direction_noise
                                       for _, _, direction in window)
                if len(values) >= 5 and stamp-window[0][0] >= HOLD_SECONDS and max(values)-min(values) <= 2*self.noise and direction_stable:
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
                      offset_mm=offset, target_mm=target, metric='shaft_to_tip_3d',
                      reference_angle_3d_deg=self.reference_angle, peak_threshold_deg=self.noise)
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
        triggers, angles, holding_angle = summarize(frames, events, self.reference_angle, self.noise)
        result.update(status='captured', triggers=triggers, target_hold_mean_3d_deg=holding_angle,
                      duration_s=events['T2'] - events['T0'], frame_count=len(frames),
                      valid_3d_frames=sum(angle is not None for angle in angles))
        save_json(path / 'round.json', result)
        result.update(status='complete', videos=export_videos(path, frames, angles, triggers))
        save_json(path / 'round.json', result)
        self.completed += 1
        peak = triggers['T1'].get('angle_3d_deg')
        status = triggers['T1']['status']
        self.progress('round_complete', last_peak_deg=peak, last_peak_status=status, last_round=str(path))
        reading = f'{peak:.2f}°' if peak is not None else '无有效三维帧'
        print(f'点 {point} · 第 {repeat} 轮 · {result["duration_s"]:.1f}s · 三维峰值 {reading} · {path}', flush=True)
        return dict(path=path.name, segment=self.segment, peak_3d_deg=peak, peak_status=status,
                    residual_deg=triggers['T2']['change_from_reference_deg'])

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
            raise RuntimeError('Pre-test 需要有效三维角、静止电缸和各轴 0.5 mm/s 的速度配置')
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
                    peaks = [r['peak_3d_deg'] for r in group['rounds'] if r['peak_3d_deg'] is not None]
                    group.update(valid_peaks=len(peaks), detected=sum(r['peak_status']=='detected' for r in group['rounds']),
                                 mean_peak_3d_deg=mean(peaks) if peaks else None,
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
