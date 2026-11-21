"""Open the macOS D405 workbench, reusing the existing camera session and service."""

import argparse
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from urllib.request import ProxyHandler, build_opener

from device_plugins.macos_camera import endpoint

ROOT = Path(__file__).resolve().parent
HTTP = build_opener(ProxyHandler({}))


def workbench_state(url):
    try:
        with HTTP.open(url + '/api/state', timeout=.5) as response:
            state = json.load(response)
        if (isinstance(state, dict) and isinstance(state.get('camera'), dict)
                and 'online' in state['camera'] and {'experiment', 'axes'} <= state.keys()):
            return state
    except (OSError, ValueError):
        pass
    return None


def bridge_listening():
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(.25)
        return client.connect_ex(str(endpoint())) == 0


def open_workbench(port, url):
    if workbench_state(url) is None:
        with tempfile.NamedTemporaryFile(prefix='ophis-workbench-', suffix='.log', delete=False) as log:
            server = subprocess.Popen([sys.executable, str(ROOT / 'server.py'), '--port', str(port)],
                                      cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 8
        while workbench_state(url) is None:
            if server.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError(f'工作台未能启动，请查看 {log.name}')
            time.sleep(.2)
    subprocess.run(['/usr/bin/open', url + '/#camera'], check=True)
    print('工作台已打开。如页面显示未接入，点击「接入 D405」。', flush=True)


def launch(port, serial):
    url = f'http://127.0.0.1:{port}'
    state = workbench_state(url)
    if (state and state['camera']['online']) or bridge_listening():
        if serial and (not state or (state['camera'].get('calibration') or {}).get('serial') != serial):
            raise RuntimeError('已有 D405 会话。切换相机时请先结束原采集终端，再指定序列号启动。')
        print('复用现有 D405 会话。', flush=True)
        open_workbench(port, url)
        return 0

    env = ROOT / '.venv'
    helper = env / 'bin' / 'ophis-d405-capture'
    source = ROOT / 'device_plugins' / 'd405_capture.cpp'
    subprocess.run(['/usr/bin/clang++', '-std=c++17', '-O2', f'-I{env / "include"}', str(source),
                    f'-L{env / "lib"}', '-lrealsense2', f'-Wl,-rpath,{env / "lib"}', '-o', str(helper)], check=True)
    bridge = subprocess.Popen(['/usr/bin/sudo', '--', str(helper), *([serial] if serial else [])])
    while not bridge_listening():
        if (code := bridge.poll()) is not None:
            return code
        time.sleep(.2)

    try:
        open_workbench(port, url)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(error, file=sys.stderr)
    print('保持此终端打开以继续使用 D405；Ctrl-C 结束相机桥接。', flush=True)
    return bridge.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('serial', nargs='?')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('端口须为 1–65535')
    if os.geteuid() == 0:
        parser.error('请以普通用户运行启动器。相机桥接会单独请求管理员权限。')
    try:
        return launch(args.port, args.serial)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
