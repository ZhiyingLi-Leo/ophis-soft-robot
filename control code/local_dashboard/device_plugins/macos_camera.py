"""Read frames from the manually authorized D405 capture session on macOS."""
import json
import os
import socket
import time
from pathlib import Path


def endpoint():
    return Path('/tmp') / f'ophis-d405-{os.getuid()}' / 'frames'


def receive(camera, serial, rs):
    import numpy as np
    with socket.socket(socket.AF_UNIX) as conn:
        conn.settimeout(5)
        conn.connect(str(endpoint()))
        with conn.makefile('rb') as stream:
            calibration = json.loads(stream.readline(8192))
            if serial and calibration['serial'] != serial:
                raise ValueError('采集会话序列号与所选 D405 不符')
            intr = rs.intrinsics()
            for name in ('depth','color'):
                spec = calibration[name]
                spec['model'] = str(rs.distortion(spec['model_id']))
            for key in ('width','height','fx','fy','ppx','ppy','coeffs'):
                setattr(intr, key, calibration['color'][key])
            intr.model = rs.distortion(calibration['color']['model_id'])
            with camera.lock:
                camera.status.update(calibration=calibration,status='等待首帧',processed_frames=0)
            while not camera.halt.is_set():
                line = stream.readline(4096)
                if not line:
                    raise RuntimeError('D405 采集会话已结束')
                info = json.loads(line)
                info['host_wall_time'] = time.time()
                arrays = []
                for shape,dtype in [((480,640,3),np.uint8),((480,640),np.uint16),((480,640),np.uint16)]:
                    size = int(np.prod(shape))*np.dtype(dtype).itemsize
                    raw = stream.read(size)
                    if len(raw)!=size:
                        raise RuntimeError('D405 帧传输中断')
                    arrays.append(np.frombuffer(raw,dtype=dtype).reshape(shape))
                camera._publish(*arrays, calibration['depth_scale_m'], intr, info, rs)
