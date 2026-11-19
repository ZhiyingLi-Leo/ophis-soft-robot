"""Record an unannotated D405 color video of ID1 reaching 5 mm and returning."""
import argparse
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import pyrealsense2 as rs

from device_plugins.macos_camera import receive
from experiment_api import Robot


class ColorVideo:
    def __init__(self, path):
        self.path = path
        self.halt = threading.Event()
        self.lock = threading.Lock()
        self.status = {}
        self.thread = threading.Thread(target=self.capture)
        self.last = None
        self.previous = None
        self.count = 0
        self.error = None
        self.writer = None

    def capture(self):
        try:
            receive(self, None, rs)
        except Exception as exc:
            self.error = exc
        finally:
            if self.writer:
                self.writer.release()

    def _publish(self, color, native, aligned, scale, intr, info, sdk):
        stamp = info['host_receive_monotonic']
        if abs(time.monotonic() - stamp) > 1:
            raise RuntimeError('D405 帧过期')
        if self.writer is None:
            self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'avc1'),
                                          15, (color.shape[1], color.shape[0]))
            if not self.writer.isOpened():
                raise RuntimeError('无法创建 MP4')
            self.start = stamp
        slot = round((stamp - self.start) * 15)
        while self.count < slot:
            self.writer.write(self.previous)
            self.count += 1
        if self.count == slot:
            self.writer.write(color)
            self.count += 1
        self.previous = color
        self.last = stamp

    def check(self):
        if self.error:
            raise self.error
        if self.last is None or time.monotonic() - self.last > 1:
            raise RuntimeError('D405 缺少新帧')


def axis_state(robot):
    axis = next(a for a in robot.state()['axes'] if a['id'] == 1)
    if not axis['online'] or axis['error'] or axis['age'] is None or axis['age'] > 1:
        raise RuntimeError('ID1 位置反馈异常')
    return axis


def run(robot):
    axis = axis_state(robot)
    if axis['moving'] or not axis['minimum'] <= 5 <= axis['maximum']:
        raise RuntimeError('请检查 ID1 状态和行程')
    start = max(axis['minimum'], axis['position'])
    if start > axis['maximum'] or axis['position'] < axis['minimum'] - .05:
        raise RuntimeError('ID1 起点超限')
    folder = Path(__file__).resolve().parents[1] / 'data'
    folder.mkdir(exist_ok=True)
    path = folder / datetime.now().strftime('D405-ID1-5mm-%Y%m%d-%H%M%S-%f.mp4')
    video = ColorVideo(path)
    video.thread.start()
    try:
        deadline = time.monotonic() + 6
        while video.count < 15 and not video.error and time.monotonic() < deadline:
            robot.wait(.1)
        video.check()
        if video.count < 15:
            raise RuntimeError('缺少开场画面')
        robot.arm(1)
        try:
            for target in (5.0, start):
                robot.move(1, target)
                deadline = time.monotonic() + 60
                while True:
                    video.check()
                    axis = axis_state(robot)
                    if not axis['armed']:
                        raise RuntimeError('ID1 启用已取消')
                    if not axis['moving'] and abs(axis['position'] - target) <= .05:
                        print(f"ID1 {axis['position']:.3f} mm", flush=True)
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('ID1 到位超时')
                    robot.wait(.15)
                robot.wait(2)
                video.check()
        finally:
            robot.stop_cylinder(1)
    finally:
        video.halt.set()
        video.thread.join()
        if path.exists():
            print(path, flush=True)
    if video.error:
        raise video.error


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    if parser.parse_args().execute:
        with Robot() as robot:
            run(robot)
    else:
        print('工作台断开 D405 预览，保留 macOS 相机采集进程。--execute 录制 ID1 到 5 mm 并返回起点。')
