"""Explicitly managed macOS side-view capture and round recorder."""

import copy
import json
import math
import os
from pathlib import Path
import queue
import select
import shutil
import subprocess
import tempfile
import threading
import time

from .vision import analyze_rgb, locate_rgb_roi, roi_pixel_bounds

HELPER = (Path(__file__).resolve().parents[1] / '.venv' / 'OPHIS Side View.app' /
          'Contents' / 'MacOS' / 'ophis-uvc-capture')
RECORD_HZ = 5
MIN_DISK_BYTES = 512 * 1024**2
MAX_FRAME_AGE = 1
FRAME_TIMEOUT = 2


def _helper():
    if not HELPER.is_file():
        raise RuntimeError('请先运行 sh device_plugins/build_uvc_helper.sh')
    return str(HELPER)


def _device_list():
    command = [_helper(), '--list']
    try:
        result = subprocess.run(command, capture_output=True, check=True, timeout=4)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(error.stderr.decode(errors='replace').strip() or '侧视设备发现失败') from error
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RuntimeError(f'侧视设备发现失败：{error}') from error
    return json.loads(result.stdout)


def _packets(pipe, halt):
    """Drain complete packets and yield only fresh, ordered frames."""
    import cv2
    import numpy as np

    fd = pipe.fileno()
    buffer = bytearray()
    deadline = time.monotonic() + 90  # Allow for the macOS camera permission dialog.
    previous_stamp, previous_number = -math.inf, -math.inf
    while not halt.is_set():
        def more():
            if halt.is_set():
                raise EOFError()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError('侧视相机等待图像超时')
            if not select.select([fd], [], [], min(.2, remaining))[0]:
                return
            chunk = os.read(fd, 65536)
            if not chunk:
                raise EOFError()
            buffer.extend(chunk)

        try:
            while (newline := buffer.find(b'\n')) < 0:
                if len(buffer) > 4096:
                    raise RuntimeError('侧视帧头无效')
                more()
            if newline > 4096:
                raise RuntimeError('侧视帧头无效')
            info = json.loads(buffer[:newline])
            del buffer[:newline+1]
            width, height, count = (info[key] for key in ('width', 'height', 'bytes'))
            if not (type(width) is type(height) is type(count) is int and
                    0 < width <= 4096 and 0 < height <= 4096 and count == width*height*4):
                raise RuntimeError('侧视帧尺寸无效')
            while len(buffer) < count:
                more()
            now = time.monotonic()
            stamp, number, fps = (info.get(k) for k in ('host_receive_monotonic', 'frame_number', 'fps'))
            if (type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp > now+.2 or
                    type(fps) not in (int, float) or not math.isfinite(fps) or fps < 0 or type(number) is not int):
                raise RuntimeError('侧视相机帧时间或格式无效')
            if stamp <= previous_stamp or number <= previous_number:
                raise RuntimeError('侧视帧序号或时间停止更新')
            if now >= deadline:
                raise RuntimeError('侧视相机等待图像超时')
            if previous_number == -math.inf:
                deadline = now + FRAME_TIMEOUT
            previous_stamp, previous_number = stamp, number
            if now-stamp > MAX_FRAME_AGE:
                del buffer[:count]
                continue
            deadline = now + FRAME_TIMEOUT
            pixels = np.frombuffer(buffer[:count], dtype=np.uint8).reshape(height, width, 4)
            del buffer[:count]
            yield info, cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        except EOFError:
            if halt.is_set():
                return
            raise RuntimeError('侧视采集进程已退出') from None


def _settings_update(settings, args):
    updated = copy.deepcopy(settings)
    if 'roi' in args:
        roi = args['roi']
        if (not isinstance(roi, list) or len(roi) != 4 or
                any(type(v) not in (float, int) or not math.isfinite(v) for v in roi) or
                not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1) or
                min(roi[2]-roi[0], roi[3]-roi[1]) < .05):
            raise ValueError('ROI 需在 0–1 内，宽高至少 0.05')
        updated['roi'] = roi.copy()
    for key in ('white_min', 'white_sat'):
        if key in args:
            value = args[key]
            if type(value) is not int or not 0 <= value <= 255:
                raise ValueError('白色阈值需要 0–255 整数')
            updated[key] = value
    for key in ('orange_min', 'orange_max'):
        if key in args:
            hsv = args[key]
            if (not isinstance(hsv, list) or len(hsv) != 3 or
                    any(type(v) is not int or not 0 <= v <= limit for v, limit in zip(hsv, (179, 255, 255)))):
                raise ValueError('橙色 HSV 范围为 0–179、0–255、0–255')
            updated[key] = hsv.copy()
    if any(lo > hi for lo, hi in zip(updated['orange_min'], updated['orange_max'])):
        raise ValueError('橙色 HSV 下限不能高于上限')
    return updated


class SideViewDevice:
    key = 'camera.side_view'
    kind = 'camera'
    operations = ('connect', 'disconnect', 'configure', 'auto_locate')

    def __init__(self, telemetry):
        self.telemetry = telemetry
        self.lock = threading.RLock()
        self.halt = threading.Event()
        self.thread = self.process = self.recording = None
        self.color = self.jpeg = None
        self.settings = dict(roi=[0., 0., 1., 1.], white_min=85, white_sat=100,
                             orange_min=[3, 85, 55], orange_max=[27, 255, 255])
        self.status = dict(online=False, status='未接入', error='', device_id=None, name=None, format=None,
                           last_frame_time=None, frame_number=None, measurement=None, measurement_settings=None,
                           recording=False, finalizing=False, recorded_frames=0, valid_recorded_frames=0,
                           last_saved_time=None, record_error='', session_path='')

    def discover(self):
        return _device_list()

    def _fresh(self, seconds=1):
        stamp = self.status['last_frame_time']
        return self.status['online'] and stamp is not None and time.monotonic()-stamp <= seconds

    def snapshot(self):
        with self.lock:
            status = copy.deepcopy(self.status)
            stamp = status['last_frame_time']
            status['frame_age'] = None if stamp is None else max(0, time.monotonic()-stamp)
            status['settings'] = copy.deepcopy(self.settings)
            return status

    def preview(self, *, with_settings=False, zoom=False):
        with self.lock:
            if not self._fresh(2) or self.jpeg is None:
                return None
            jpeg, color = self.jpeg, self.color
            settings = copy.deepcopy(self.status['measurement_settings'])
            curve = self.status['measurement']['curve_px']
        if zoom and settings['roi'] != [0, 0, 1, 1]:
            import cv2
            import numpy as np
            x0, y0, x1, y1 = roi_pixel_bounds(settings['roi'], color.shape)
            crop = color[y0:y1, x0:x1].copy()
            if curve:
                # Analysis publishes native pixel coordinates. Translate them into the crop.
                points = np.rint(np.asarray(curve) - [x0, y0]).astype(np.int32)
                cv2.polylines(crop, [points], False, (70, 220, 255), 2, cv2.LINE_AA)
                cv2.circle(crop, points[0], 4, (255, 100, 0), -1, cv2.LINE_AA)
                cv2.circle(crop, points[-1], 4, (0, 0, 255), -1, cv2.LINE_AA)
            ok, encoded = cv2.imencode('.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                return None
            jpeg = encoded.tobytes()
        return (jpeg, settings) if with_settings else jpeg

    def command(self, operation, args, context=None):
        if operation == 'connect':
            return self.connect(args.get('device_id'))
        if operation == 'disconnect':
            return self.disconnect()
        if operation == 'configure':
            with self.lock:
                self._require_configurable()
                self.settings = _settings_update(self.settings, args)
                return {'settings': copy.deepcopy(self.settings)}
        if operation == 'auto_locate':
            return self.auto_locate()
        raise ValueError('侧视相机不支持此操作')

    def _require_configurable(self):
        if self.recording is not None:
            raise ValueError('记录结束后才能修改侧视 ROI 和颜色阈值')

    def auto_locate(self):
        return self.apply_auto_roi(self.roi_candidate())

    def roi_candidate(self):
        with self.lock:
            self._require_configurable()
            if not self._fresh() or self.color is None:
                raise ValueError('请先接入侧视相机并等待新鲜图像')
            settings, color, connection = self.settings, self.color, self.halt
        return settings, connection, locate_rgb_roi(color, settings)

    def apply_auto_roi(self, candidate):
        settings, connection, roi = candidate
        with self.lock:
            self._require_configurable()
            if settings is not self.settings or connection is not self.halt or not self._fresh():
                raise ValueError('侧视相机或设置已变化，请重新自动定位')
            self.settings = dict(settings, roi=roi)
            return {'settings': copy.deepcopy(self.settings)}

    def connect(self, device_id):
        if (not isinstance(device_id, str) or not 0 < len(device_id) <= 512 or
                any(ord(c) < 32 for c in device_id)):
            raise ValueError('请选择一个侧视摄像头')
        with self.lock:
            if self.thread and self.thread.is_alive():
                raise ValueError('侧视相机正在连接或断开，请等待')
            if self.recording:
                raise ValueError('侧视记录正在结束，请等待')
            self.halt = threading.Event()
            self.color = self.jpeg = None
            self.status.update(online=False, status='连接中', error='', name=None, device_id=device_id, format=None,
                               last_frame_time=None, frame_number=None, measurement=None, measurement_settings=None)
            self.thread = threading.Thread(target=self._capture, args=(device_id,), daemon=True)
            self.thread.start()

    def disconnect(self):
        with self.lock:
            self.halt.set()
            self.status.update(online=False, status='未接入', error='', measurement=None, last_frame_time=None)
            self.color = self.jpeg = None
            if self.process is not None:
                self.process.terminate()
            self.stop_recording('侧视相机断开')

    def close(self):
        self.disconnect()
        with self.lock:
            capture, rec = self.thread, self.recording
        if capture:
            capture.join(timeout=3)
        if rec:
            rec['thread'].join(timeout=3)

    def _capture(self, device_id):
        process = None
        try:
            import cv2  # Load OpenCV before the helper can start writing frames.

            device = next((item for item in self.discover() if item['id'] == device_id), None)
            if device is None:
                raise RuntimeError('所选侧视摄像头已离线，请刷新设备列表')
            with tempfile.TemporaryFile() as errors:
                with self.lock:
                    if self.halt.is_set():
                        return
                    process = subprocess.Popen([_helper(), '--capture', device_id], stdout=subprocess.PIPE,
                                               stderr=errors, stdin=subprocess.DEVNULL)
                    self.process = process
                    self.status.update(name=device['name'], status='等待侧视首帧')
                try:
                    self._stream(process, cv2)
                except Exception as error:
                    errors.seek(0)
                    message = errors.read(8192).decode(errors='replace').strip()
                    raise RuntimeError(f'{error} · {message}' if message else str(error)) from error
        except Exception as error:
            with self.lock:
                if not self.halt.is_set():
                    self.status.update(error=str(error), status='侧视采集中断')
                    self.stop_recording(str(error), failed=True)
        finally:
            if process is not None:
                process.kill()
                process.wait()
                process.stdout.close()
            with self.lock:
                self.process = None
                self.status.update(online=False, measurement=None)
                self.color = self.jpeg = None

    def _stream(self, process, cv2):
        available = threading.Condition()
        finished = threading.Event()
        latest = None
        processing_error = None

        def process_latest():
            nonlocal latest, processing_error
            try:
                while True:
                    with available:
                        available.wait_for(lambda: latest is not None or finished.is_set())
                        if finished.is_set():
                            return
                        info, color = latest
                        latest = None
                    if time.monotonic() - info['host_receive_monotonic'] <= MAX_FRAME_AGE:
                        self._publish(info, color, cv2=cv2, finished=finished)
            except Exception as error:
                processing_error = error
                process.terminate()  # Wake the pipe reader so it can report the processing error.

        worker = threading.Thread(target=process_latest, daemon=True)
        worker.start()
        try:
            for frame in _packets(process.stdout, self.halt):
                with available:
                    latest = frame
                    available.notify()
        except Exception:
            if processing_error is not None:
                raise processing_error
            raise
        finally:
            with self.lock:
                finished.set()
            with available:
                available.notify()
            worker.join(timeout=.2)

    def _publish(self, info, color, *, cv2=None, finished=None):
        if cv2 is None:
            import cv2

        stamp, number = info['host_receive_monotonic'], info['frame_number']
        fps = info['fps']
        with self.lock:
            settings = self.settings
            rec = self.recording
        feedback = self.telemetry() if rec else None
        measurement, overlay = analyze_rgb(color, settings)
        ok, jpeg = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if not ok:
            raise RuntimeError('侧视预览编码失败')
        with self.lock:
            if (self.halt.is_set() or (finished is not None and finished.is_set()) or
                    time.monotonic()-stamp > MAX_FRAME_AGE):
                return
            height, width = color.shape[:2]
            stream_format = dict(width=width, height=height, fps=fps or None)
            self.status.update(online=True, status='采集中', error='', last_frame_time=stamp,
                               frame_number=number, format=stream_format, measurement=measurement,
                               measurement_settings=settings)
            self.color, self.jpeg = color, jpeg.tobytes()
            if rec is not None and rec is self.recording and not rec['stop'].is_set():
                if (width, height) != rec['size']:
                    self.stop_recording('侧视相机录制格式已改变', failed=True)
                elif stamp >= rec['started'] and stamp >= rec['next_sample']:
                    self._enqueue(rec, color, dict(kind='frame', frame_number=number,
                        host_receive_monotonic=stamp, fps=stream_format['fps'], measurement=measurement, telemetry=feedback))

    def record_start(self, round_path):
        with self.lock:
            if self.recording or not self._fresh():
                raise ValueError('侧视相机须有新鲜图像且没有进行中的记录')
            if self.status['measurement_settings'] != self.settings:
                raise ValueError('侧视相机正在应用识别设置，请等待新鲜预览')
            if shutil.disk_usage(round_path).free < MIN_DISK_BYTES:
                raise OSError('可用空间不足 512 MB')
            path = Path(round_path) / 'side_view'
            path.mkdir()
            stream_format = self.status['format']
            metadata = dict(settings=self.settings, format=stream_format, name=self.status['name'],
                            device_id=self.status['device_id'], target_record_hz=RECORD_HZ,
                            timing_note='host receive time is not exposure time',
                            event_index='../frames.jsonl')
            (path/'metadata.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8')
            index = (path/'frames.jsonl').open('w', encoding='utf-8')
            rec = dict(path=path, queue=queue.Queue(maxsize=4), stop=threading.Event(),
                       started=time.monotonic(), next_sample=-math.inf, reason='', size=(stream_format['width'],stream_format['height']))
            rec['thread'] = threading.Thread(target=self._write, args=(rec, index), daemon=True)
            self.recording = rec
            self.status.update(recording=True, finalizing=False, recorded_frames=0, valid_recorded_frames=0,
                               last_saved_time=None, record_error='', session_path=str(path))
            try:
                rec['thread'].start()
            except RuntimeError:
                self.recording = None
                self.status['recording'] = False
                index.close()
                raise

    def _enqueue(self, rec, color, info):
        try:
            rec['queue'].put_nowait((color, info))
        except queue.Full:
            self.stop_recording('侧视写入队列已满', failed=True)
        else:
            stamp = info['host_receive_monotonic']
            period = 1/RECORD_HZ
            previous = rec['next_sample']
            rec['next_sample'] = max(previous+period,stamp+period/2) if math.isfinite(previous) else stamp+period-.005

    def stop_recording(self, reason, *, failed=False):
        with self.lock:
            rec = self.recording
            if rec is None:
                return
            if failed:
                self.status['record_error'] = reason
            if not rec['stop'].is_set() or failed:
                rec['reason'] = reason
            self.status.update(recording=False, finalizing=True)
            rec['stop'].set()

    def _write(self, rec, index):
        import cv2

        count = 0
        try:
            with index:
                try:
                    while not rec['stop'].is_set() or not rec['queue'].empty():
                        try:
                            color, info = rec['queue'].get(timeout=.05)
                        except queue.Empty:
                            continue
                        if shutil.disk_usage(rec['path']).free < MIN_DISK_BYTES:
                            raise OSError('可用空间不足 512 MB')
                        name = f'frame_{count:06d}.png'
                        pending = rec['path']/f'partial_{name}'
                        if not cv2.imwrite(str(pending), color, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                            raise OSError('侧视 PNG 写入失败')
                        pending.replace(rec['path']/name)
                        index.write(json.dumps(dict(info, file=name), ensure_ascii=False, allow_nan=False)+'\n')
                        index.flush()
                        count += 1
                        with self.lock:
                            self.status.update(recorded_frames=count, last_saved_time=time.monotonic())
                            self.status['valid_recorded_frames'] += int(info['measurement']['valid'])
                except Exception as error:
                    self.stop_recording(str(error), failed=True)
                index.write(json.dumps(dict(kind='end', reason=rec['reason'], frames=count,
                                            host_monotonic=time.monotonic()), ensure_ascii=False)+'\n')
        except Exception as error:
            self.stop_recording(str(error), failed=True)
        finally:
            with self.lock:
                self.recording = None
                self.status.update(recording=False, finalizing=False)
