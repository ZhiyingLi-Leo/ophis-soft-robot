"""Acquire a bounded single-, two- or three-cylinder sweep from 0 mm."""

import argparse
import math
import time
from datetime import datetime
from itertools import product
from pathlib import Path
from statistics import median

from experiment_api import Robot
from pretest_data import frame_angle, read_capture, save_json, shape_angle, summarize
from sampling import ArrivalWindow, axes_by_id, positions_at_target, require_camera

REFERENCE = (0, 0, 0)
REPEATS = 3
REFERENCE_SECONDS = 4
HOLD_SECONDS = 4
RETURN_SECONDS = 2
POSITION_TOLERANCE = .05
SPEED = .5


class AxisCapture:
    def __init__(self, robot, ids, points, max_advance):
        self.robot = robot
        self.ids = ids
        self.group = 'axis_capture_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.batch = dict(ids=ids, points_mm=points, repeats=REPEATS, reference_mm=REFERENCE,
                          max_advance_mm=max_advance, reference_seconds=REFERENCE_SECONDS,
                          hold_seconds=HOLD_SECONDS, return_seconds=RETURN_SECONDS, completed_rounds=[])

    def feedback(self, target, *, armed, recording=False):
        state = self.robot.state()
        camera = require_camera(state, recording=recording)
        axes = axes_by_id(state)
        if camera['settings'] != self.settings or camera['measurement_settings'] != self.settings:
            raise RuntimeError('相机识别参数已变化')
        angle = shape_angle(camera.get('measurement'))
        now = time.monotonic()
        if angle is not None:
            self.last_valid = now
        elif now - self.last_valid > 1:
            reason = ((camera.get('measurement') or {}).get('shape_3d') or {}).get('reason', '')
            raise RuntimeError('有效三维角持续丢失。' + reason)

        for axis, upper in zip(axes, target):
            axis_id = axis['id']
            enabled = armed and axis_id in self.ids
            if not axis['online'] or axis['error'] or axis['age'] is None or axis['age'] > 1:
                raise RuntimeError(f'ID{axis_id} 反馈异常或过期')
            if axis['armed'] != enabled or (not enabled and axis['moving']):
                raise RuntimeError(f'ID{axis_id} 的启用或运动状态发生变化')
            if not -POSITION_TOLERANCE <= axis['position'] <= upper + POSITION_TOLERANCE:
                raise RuntimeError(f'ID{axis_id} 的实际位置超出本轮位移范围')
        return axes, camera

    def move(self, destination):
        if len(self.ids) == 1:
            self.robot.move(self.slots[0], destination[self.ids[0] - 1])
        elif len(self.ids) == 2:
            self.robot.move_group(self.slots, [destination[i - 1] for i in self.ids])
        else:
            self.robot.move_all([destination[i - 1] for i in self.slot_ids])

    def wait_arrival(self, destination, target):
        # ID1's measured 1–5 mm moves took longer than displacement / configured speed.
        timeout = max(30, 10 + 2.5 * max(target) / SPEED)
        deadline = time.monotonic() + timeout
        window = ArrivalWindow(destination, POSITION_TOLERANCE)
        while time.monotonic() < deadline:
            axes, _ = self.feedback(target, armed=True, recording=True)
            if window.update(axes):
                return
            self.robot.wait(.08)
        raise RuntimeError(f'位置 {destination} 到位超时')

    def hold(self, destination, target, seconds):
        deadline = time.monotonic() + seconds
        while True:
            axes, _ = self.feedback(target, armed=True, recording=True)
            if not positions_at_target(axes, destination, POSITION_TOLERANCE):
                raise RuntimeError(f'在 {destination} 保持期间，实际位置发生偏移')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self.robot.wait(min(.08, remaining))

    def wait_first_frame(self):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            _, camera = self.feedback(REFERENCE, armed=True, recording=True)
            if camera['recorded_frames'] and shape_angle(camera.get('measurement')) is not None:
                return
            self.robot.wait(.06)
        raise RuntimeError('参考位未收到有效画面或未写入原始帧')

    def stop(self):
        for slot in self.slots:
            try:
                self.robot.stop_cylinder(slot)
            except RuntimeError as error:
                # The experiment runner also stops all devices when this process exits.
                print(f'停止请求异常：{error}', flush=True)

    def capture_round(self, point, repeat, target):
        label = f'point_{point:03d}_repeat_{repeat}'
        result = self.robot.device('camera.d405', 'record_start', label=label, group=self.group)
        path = Path(result['session_path'])
        round_info = dict(status='incomplete', point=point, repeat=repeat, ids=self.ids, target_mm=target,
                          reference_mm=REFERENCE)
        try:
            save_json(path / 'round.json', round_info)
            save_json(path.parent / 'points.json', self.batch)
            self.wait_first_frame()
            self.robot.annotate(kind='reference_hold')
            self.hold(REFERENCE, REFERENCE, REFERENCE_SECONDS)
            self.robot.annotate(kind='T0', point=point, repeat=repeat, target=target)
            self.move(target)
            self.wait_arrival(target, target)
            self.robot.annotate(kind='target_hold')
            self.hold(target, target, HOLD_SECONDS)
            self.robot.annotate(kind='return')
            self.move(REFERENCE)
            self.wait_arrival(REFERENCE, target)
            self.hold(REFERENCE, target, RETURN_SECONDS)
            self.robot.annotate(kind='T2')
        except BaseException:
            self.stop()
            raise
        finally:
            self.robot.record_stop()

        frames, events = read_capture(path)
        reference_frames = [f for f in frames if events['reference_hold'] <= f['host_receive_monotonic'] < events['T0']]
        hold_frames = [f for f in frames if events['target_hold'] <= f['host_receive_monotonic'] < events['return']]
        if not reference_frames or not hold_frames:
            raise RuntimeError(f'参考段或目标停留段缺少原始帧：{path}')
        reference = [angle for f in reference_frames if (angle := frame_angle(f)) is not None]
        held = [angle for f in hold_frames if (angle := frame_angle(f)) is not None]
        round_info.update(status='complete', events_monotonic_s=events, frame_count=len(frames),
                          reference_frames=len(reference_frames), hold_frames=len(hold_frames),
                          valid_reference_angles=len(reference), valid_hold_angles=len(held))
        if reference:
            zero = median(reference)
            deviations = sorted(abs(angle - zero) for angle in reference)
            noise = max(.5, 2 * deviations[math.ceil(.95 * len(reference)) - 1])
            triggers, _, _ = summarize(frames, events, zero, noise)
            round_info['preview'] = dict(reference_angle_deg=zero, peak_threshold_deg=noise, triggers=triggers,
                                         hold_delta_median_deg=median(held) - zero if held else None)
        save_json(path / 'round.json', round_info)
        self.batch['completed_rounds'].append(dict(point=point, repeat=repeat, path=path.name))
        save_json(path.parent / 'points.json', self.batch)
        return path.parent

    def run(self):
        state = self.robot.state()
        camera = require_camera(state)
        if camera['recording'] or shape_angle(camera.get('measurement')) is None:
            raise RuntimeError('请先结束现有录制，并在相机页确认三维识别有效')
        self.settings = camera['settings']
        self.last_valid = time.monotonic()
        axes, _ = self.feedback(REFERENCE, armed=False)
        self.slot_ids = [axis['id'] for axis in state['axes']]
        self.slots = [self.slot_ids.index(axis_id) + 1 for axis_id in self.ids]
        for axis, max_target in zip(axes, (max(values) for values in zip(*self.batch['points_mm']))):
            if not axis['minimum'] <= 0 <= max_target <= axis['maximum']:
                raise RuntimeError(f'ID{axis["id"]} 的参考位或目标超出装配限位')
            if axis['id'] in self.ids and not math.isclose(axis['speed'], SPEED):
                raise RuntimeError(f'ID{axis["id"]} 需要 0.5 mm/s 的速度配置')

        points = self.batch['points_mm']
        total = len(points) * REPEATS
        try:
            for slot in self.slots:
                self.robot.arm(slot)
            for repeat in range(1, REPEATS + 1):
                for point, target in enumerate(points, 1):
                    number = len(self.batch['completed_rounds']) + 1
                    print(f'采集 {number}/{total} · 第 {repeat} 遍 · ID1/2/3 目标 {target} mm', flush=True)
                    directory = self.capture_round(point, repeat, target)
                    print(f'已回参考位并保存 {number}/{total} · {directory}', flush=True)
        finally:
            self.stop()
        print(f'本段 {total} 轮完成 · {directory / "points.json"}', flush=True)


def main(channels):
    parser = argparse.ArgumentParser(description=f'{channels} 轴位移扫点，每点三轮并保存原始帧')
    if channels < 3:
        parser.add_argument('--ids', nargs=channels, required=True, type=int, choices=(1, 2, 3), metavar='ID')
    parser.add_argument('--start-mm', type=int, help='本段第一个目标，默认为步长')
    parser.add_argument('--stop-mm', type=int, required=True, help='本段最后一个目标')
    parser.add_argument('--step-mm', type=int, default=1 if channels == 1 else 2)
    parser.add_argument('--max-advance-mm', type=float, help='现场确认的选中各轴总推进量上限')
    parser.add_argument('--execute', action='store_true', help='启动采集，省略时预览计划')
    args = parser.parse_args()
    ids = tuple(args.ids) if channels < 3 else (1, 2, 3)
    if len(set(ids)) != channels:
        parser.error('电缸 ID 必须各不相同')
    start = args.start_mm if args.start_mm is not None else args.step_mm
    if not 1 <= start <= args.stop_mm <= 10 or args.step_mm <= 0 or (args.stop_mm - start) % args.step_mm:
        parser.error('目标须在 1–10 mm 内，末点须落在扫描步长上')
    if args.max_advance_mm is not None and (not math.isfinite(args.max_advance_mm)
                                           or args.max_advance_mm < channels * args.stop_mm):
        parser.error('目标网格的最大组合位移超过本次总推进量上限')
    if args.execute and args.max_advance_mm is None:
        parser.error('开始采集前须用 --max-advance-mm 填写现场确认的总推进量上限')
    levels = range(start, args.stop_mm + 1, args.step_mm)
    points = []
    for values in product(levels, repeat=channels):
        offsets = dict(zip(ids, values))
        points.append(tuple(offsets.get(i, 0) for i in (1, 2, 3)))
    print(f'ID {ids} · {len(points)} 个位移点 · {REPEATS} 遍 · 共 {len(points) * REPEATS} 轮', flush=True)
    if args.execute:
        with Robot() as robot:
            AxisCapture(robot, ids, points, args.max_advance_mm).run()
