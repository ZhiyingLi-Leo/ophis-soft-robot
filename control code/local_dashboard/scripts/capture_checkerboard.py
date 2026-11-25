"""在实验台录制双机棋盘格，或对已保存的会话执行立体标定。"""

import argparse
from datetime import datetime
from math import ceil, isfinite
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiment_api import Robot
from sampling import require_stream
from stereo_calibration import (
    DEFAULT_CORNERS, DEFAULT_MAX_DT_MS, DEFAULT_MIN_PAIRS, InsufficientPairsError, calibrate_session,
)

GUIDANCE = '请将棋盘格斜约 45° 置于双机共同视野内，缓慢变换 15–20 个姿态，每个姿态停稳 1–2 秒。'


def _check_cameras(state, *, recording=False):
    for key, name in (('camera.d405', 'D405'), ('camera.side_view', 'Ace Pro')):
        stream = state['modules'].get(key, {}).get('state')
        camera = require_stream(stream, recording=recording, name=name)
        if not recording and (camera['recording'] or camera.get('finalizing')):
            raise RuntimeError(f'{name} 正在录制或收尾，请先结束当前记录')


def capture_session(robot, seconds):
    _check_cameras(robot.state())
    group = datetime.now().strftime('stereo_calib_%Y%m%d_%H%M%S_%f')
    try:
        result = robot.axis_record_start(label='checkerboard', group=group)
        directory = Path(result['session_path'])
        print(f'记录目录：{directory}', flush=True)
        print(GUIDANCE, flush=True)
        deadline = time.monotonic() + seconds
        next_countdown = 0
        while (now := time.monotonic()) < deadline:
            _check_cameras(robot.state(), recording=True)
            if now >= next_countdown:
                print(f'剩余 {ceil(deadline - now)} 秒，请停稳后再换姿态。', flush=True)
                next_countdown = now + 5
            robot.wait(min(1, max(0, deadline - time.monotonic())))
    finally:
        capturing_failed = sys.exc_info()[0] is not None
        try:
            robot.axis_record_stop()
        except Exception as error:
            if not capturing_failed:
                raise
            print(f'结束记录时收到错误：{error}', file=sys.stderr, flush=True)
    return directory


def print_counts(counts):
    print(f"时间配对 {counts['paired_frames']} 帧，双机检出 {counts['detected_pairs']} 对，"
          f"选用不同姿态 {counts['selected_poses']} 组。")
    print(f"仅 D405 检出 {counts['d405_only']} 帧，仅 Ace Pro 检出 {counts['side_only']} 帧。")
    if counts['ambiguous_pairs']:
        print(f"已跳过朝向无法可靠配对的图像 {counts['ambiguous_pairs']} 对。")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, help='已有双机记录目录（内有 metadata.json 与 side_view/）')
    parser.add_argument('--square-mm', type=float, required=True, help='方格边长，A 板 10 mm，B 板 8 mm')
    parser.add_argument('--seconds', type=float, default=45, help='录制时长，默认 45 秒')
    parser.add_argument('--inner-corners', type=int, nargs=2, metavar=('COLS', 'ROWS'),
                        default=DEFAULT_CORNERS, help='横纵内角点数，默认 7 5')
    parser.add_argument('--max-dt-ms', type=float, default=DEFAULT_MAX_DT_MS, help='最大配对时间差，默认 120 ms')
    parser.add_argument('--min-pairs', type=int, default=DEFAULT_MIN_PAIRS, help='最少不同姿态数，默认 8')
    args = parser.parse_args(argv)
    if not isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error('--square-mm 须为正数')
    if not isfinite(args.max_dt_ms) or args.max_dt_ms <= 0:
        parser.error('--max-dt-ms 须为正数')
    if args.min_pairs < 3:
        parser.error('--min-pairs 须至少为 3')
    if min(args.inner_corners) < 3 or args.inner_corners[0] == args.inner_corners[1]:
        parser.error('--inner-corners 须为两个不同且至少为 3 的整数')
    if args.session is None and (not isfinite(args.seconds) or args.seconds <= 0):
        parser.error('--seconds 须为正数')

    directory = args.session
    try:
        if directory is None:
            if not all(os.environ.get(key) for key in ('OPHIS_URL', 'OPHIS_TOKEN')):
                parser.error('请在已连接双机的实验台运行录制，或使用 --session 离线标定')
            with Robot() as robot:
                directory = capture_session(robot, args.seconds)
        print(f'正在分析双机图像：{directory}', flush=True)
        result = calibrate_session(directory, args.square_mm, inner_corners=tuple(args.inner_corners),
                                   max_dt_ms=args.max_dt_ms, min_pairs=args.min_pairs)
    except InsufficientPairsError as error:
        print_counts(error.counts)
        print(f'标定未完成：{error}。原始记录保留在 {directory}。')
        print('请检查两机都能看清棋盘四边与全部内角点，减少反光和运动模糊；'
              '同时改变棋盘的倾角、位置与距离。')
        return 1
    except KeyboardInterrupt:
        print('操作已中断。', file=sys.stderr)
        return 130
    except Exception as error:
        print(f'操作失败：{error}', file=sys.stderr)
        return 1

    print_counts(result['counts'])
    print(f"Ace Pro 内参 RMS {result['ace_pro_rms_px']:.3f} px，"
          f"立体重投影 RMS {result['stereo_rms_px']:.3f} px，"
          f"基线 {result['baseline_mm']:.2f} mm。")
    print(f'标定结果：{(directory / "stereo_calibration.json").resolve()}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
