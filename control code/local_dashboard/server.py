#!/usr/bin/env python3
"""Loopback-only dashboard. Hardware disconnected by default; no hardware opens at startup."""
import argparse
import copy
import glob
import json
import math
import os
import select
import signal
import struct
import termios
import fcntl
import threading
import time
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

from experiments import Experiments
from pretest_data import shape_angle
from device_plugins import DeviceRegistry, CylinderDevice, PropulsionDevice
from device_plugins.realsense import D405Device
from device_plugins.uvc import SideViewDevice

ROOT = Path(__file__).resolve().parent

def frame(device, op, payload=b''):
    data = bytes([0x55, 0xaa, len(payload)+1, device, op]) + payload
    return data + bytes([sum(data[2:]) & 255])

class Serial:
    def __init__(self, path, baud=115200):
        self.last_exchange = 0.
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = getattr(termios, 'B'+str(baud))
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            try: fcntl.ioctl(self.fd, termios.TIOCNXCL)
            finally: os.close(self.fd); self.fd = None

    def exchange(self, request, decoder, timeout=.18, gap=.01):
        pause=gap-(time.monotonic()-self.last_exchange)
        if pause>0: time.sleep(pause)
        self.last_exchange=time.monotonic()
        termios.tcflush(self.fd, termios.TCIFLUSH)
        deadline = time.monotonic() + timeout
        pending = request
        while pending:
            if time.monotonic() >= deadline: raise TimeoutError('串口写超时')
            if select.select([], [self.fd], [], .01)[1]:
                try: pending = pending[os.write(self.fd, pending):]
                except BlockingIOError: pass
        data = bytearray()
        while time.monotonic() < deadline:
            if select.select([self.fd], [], [], .01)[0]:
                chunk = os.read(self.fd, 512)
                if not chunk: raise ConnectionError('串口断开')
                data.extend(chunk)
                result = decoder(data)
                if result is not None:
                    self.last_exchange=time.monotonic()
                    return result
                if len(data) > 4096: break
        self.last_exchange=time.monotonic()
        raise TimeoutError(f'未收到有效回复，接收 {len(data)} 字节，hex={data[:32].hex()}')

    def cylinder(self, device, address=None, values=()):
        op = 0x30 if address is None else 0x32
        addr = address or 0
        payload = b'' if address is None else struct.pack('<'+'H'*(1+len(values)), address, *values)
        def decode(data):
            for i in range(len(data)-19):
                p = data[i:i+20]
                if p[:5] != bytes([0xaa, 0x55, 15, device, op]): continue
                if int.from_bytes(p[5:7], 'little') != addr or sum(p[2:19]) & 255 != p[19]: continue
                target, position = struct.unpack('<hh', p[7:11])
                if p[18]: raise RuntimeError(f'ID {device} 故障码 {p[18]}')
                return dict(position=position, target=target, temperature=p[17])
        return self.exchange(frame(device, op, payload), decode, gap=.05)

    def line(self, command, prefix=None):
        def decode(data):
            for line in bytes(data).split(b'\n')[:-1]:
                text = line.decode('ascii', errors='replace').strip()
                if text.startswith('ERR'): raise RuntimeError(text)
                if prefix is None or text.startswith(prefix): return text
        return self.exchange((command+'\n').encode(), decode, .25)

def configuration(path):
    values = {}
    for line in path.read_text().splitlines():
        line = line.split('#')[0].strip()
        if line:
            key, value = line.split('=', 1); values[key.strip()] = value.strip()
    axes = []
    for n in range(1, 4):
        fields = values['cylinder'+str(n)].split(',')
        device, stroke, low, high, speed = map(float, fields)
        if not all(math.isfinite(v) for v in (device, stroke, low, high, speed)):
            raise ValueError('电缸参数必须是有限数值')
        if not (device.is_integer() and 1 <= device <= 254 and 0 <= low < high <= stroke and speed > 0 and 1 <= speed/stroke*2000 <= 32767 and math.ceil(low/stroke*2000) <= math.floor(high/stroke*2000)):
            raise ValueError('电缸参数无效')
        axes.append(dict(id=int(device), stroke=stroke, minimum=low, maximum=high, speed=speed))
    if len({a['id'] for a in axes}) != 3: raise ValueError('电缸ID不能重复')
    tolerance = float(values.get('zero_tolerance_mm', 0))
    if not 0 <= tolerance <= .05: raise ValueError('零位容差无效')
    return axes, int(values.get('cylinder_baud', 115200)), tolerance

class Engine:
    def __init__(self, config, *, testing=False):
        self.params, self.baud, self.tolerance = configuration(config)
        self.lock = threading.RLock()
        self.testing = testing
        self.mode = 'simulation' if testing else 'hardware'
        self.axes = [dict(p, online=False, armed=False, position=0., target=0., command=0., moving=False,
                          error='', temperature=None, last_seen=0., owned=False, comm_failures=0, comm_recoveries=0, last_comm_error='') for p in self.params]
        self.prop = dict(online=False, armed=False, speed=0, error='', backend='', port='', motors=[], telemetry_time=0.)
        self.prop.update(targets=[0]*4, deadlines=[0.]*4)
        self._telemetry_cache=dict(valid=False,reason='no feedback yet',host_monotonic=time.monotonic())
        self.camera=D405Device(ROOT/'data',self.telemetry_snapshot)
        self.side_view = SideViewDevice(self.telemetry_snapshot)
        self.plugins=DeviceRegistry([*(CylinderDevice(self,i) for i in range(3)),PropulsionDevice(self),
                                     self.camera,self.side_view])
        self.experiment=Experiments(self)
        self.buses = {}; self.axis_ports = {}; self.prop_serial = None
        self.devices = []; self.events = []; self.owner = None; self.heartbeat = 0
        self.heartbeat_seen=('',0.)
        self.stop_reason=''
        self.heartbeat_count=0
        self.gamepad = dict(active=False, deadline=0., axes=[], propulsion=False)
        self.gamepad_moving=set()
        self.last_scan = 0.; self.inventory = None; self.scan_enabled = False; self.running = True
        self.log('控制台已启动 · '+self.mode+' · 所有模块独立接入')

    def log(self, text):
        self.events.insert(0, dict(time=time.strftime('%H:%M:%S'), text=text))
        del self.events[80:]

    def state(self):
        with self.lock:
            axes = copy.deepcopy(self.axes)
            for a in axes: a['age'] = round(time.monotonic()-a['last_seen'], 2) if a['last_seen'] else None
            prop = copy.deepcopy(self.prop)
            prop['telemetry_age'] = round(time.monotonic()-prop.pop('telemetry_time'), 2) if prop['telemetry_time'] else None
            return dict(mode=self.mode, axes=axes, propulsion=prop, devices=copy.deepcopy(self.devices),
                        experiment=self.experiment.state(), camera=self.camera.snapshot(), modules=self.plugins.snapshot(),
                        gamepad=dict(self.gamepad), owner=self.owner, tolerance=self.tolerance, stop_reason=self.stop_reason, control_link=dict(age=round(time.monotonic()-self.heartbeat_time(),3) if self.owner else None, received=self.heartbeat_count))

    def telemetry_snapshot(self):
        if not self.lock.acquire(blocking=False):
            return dict(copy.deepcopy(self._telemetry_cache),snapshot_reused=True)
        try:
            return dict(valid=True,host_monotonic=time.monotonic(),axes=copy.deepcopy(self.axes),propulsion=copy.deepcopy(self.prop))
        finally:self.lock.release()

    def stop_axis(self, i):
        a = self.axes[i]
        a['armed'] = a['moving'] = False
        a['target'] = max(a['minimum'], min(a['maximum'], a['position']))
        if a['owned'] and self.mode == 'hardware':
            try: self.buses[self.axis_ports[i]].cylinder(a['id'], 0x1a, (1,))
            except Exception as e:
                a['error'] = '停止未确认: '+str(e)
                self.log(f'电缸 {i+1} {a["error"]}')
                return
        a['owned'] = False

    def stop_prop(self):
        self.gamepad['propulsion']=False
        self.prop['speed'] = 0; self.prop['armed'] = False
        self.prop['targets']=[0]*4; self.prop['deadlines']=[0.]*4
        if self.prop_serial:
            try:
                self.prop_serial.line('V0', 'OK V0')
                if self.prop['backend'] == 'pcb': self.prop_serial.line('A0', 'OK A0')
            except Exception as e: self.prop['error'] = '停止未确认: '+str(e); self.log(self.prop['error'])

    def stop(self, reason='停止请求'):
        self.gamepad['active']=False
        self.gamepad_moving.clear()
        self.stop_reason=reason
        for i in range(3): self.stop_axis(i)
        self.stop_prop()
        self.side_view.stop_recording(reason)
        self.camera.stop_recording(reason)

    def disconnect(self):
        self.stop()
        for port in self.buses.values(): port.close()
        self.buses.clear(); self.axis_ports.clear()
        if self.prop_serial: self.prop_serial.close(); self.prop_serial = None
        for a in self.axes: a['online'] = False; a['last_seen'] = 0
        self.prop['online'] = False

    @staticmethod
    def ports():
        return sorted(set(p for pattern in ('/dev/cu.usbserial*', '/dev/cu.usbmodem*', '/dev/cu.SLAB_USBtoUART*', '/dev/cu.wchusbserial*') for p in glob.glob(pattern)))

    def scan(self):
        if self.mode != 'hardware':
            self.devices = [dict(port='sim://cylinders', kind='cylinders', ids=[a['id'] for a in self.axes]),
                            dict(port='sim://propulsion', kind='pcb', ids=[])]
            return
        if any(a['armed'] for a in self.axes) or self.prop['armed']: raise ValueError('请停止并撤销启用后再扫描')
        paths = self.ports(); self.inventory = paths
        connected = set(self.buses) | ({self.prop['port']} if self.prop_serial else set())
        devices = [d for d in self.devices if d['port'] in connected]
        for path in paths:
            if path in connected: continue
            port = None
            try:
                port = Serial(path, self.baud)
                ids = []
                for a in self.axes:
                    try: port.cylinder(a['id']); ids.append(a['id'])
                    except Exception: pass
                if ids: devices.append(dict(port=path, kind='cylinders', ids=ids)); continue
                try:
                    port.line('INFO', 'INFO PCB1_S3 ')
                    devices.append(dict(port=path, kind='pcb', ids=[])); continue
                except Exception: pass
                # Opening a Nano can reset it. Wait before its read-only identity query.
                time.sleep(2)
                try:
                    reply = port.line('?', 'V'); int(reply[1:])
                    devices.append(dict(port=path, kind='nano', ids=[]))
                except Exception: devices.append(dict(port=path, kind='unknown', ids=[]))
            except Exception as e: devices.append(dict(port=path, kind='unavailable', ids=[], error=str(e)))
            finally:
                if port: port.close()
        self.devices = devices; self.last_scan = time.monotonic()

    def connect_axis(self, i, path):
        if not 0 <= i < 3: raise ValueError('轴编号无效')
        a = self.axes[i]
        if a['online']: return
        if self.mode == 'simulation':
            a['position'] = max(a['minimum'], min(a['maximum'], a['position']))
            a['command'] = a['target'] = a['position']
            a.update(online=True, armed=False, error='', comm_failures=0, last_seen=time.monotonic(), temperature=28)
            return
        matches = [d for d in self.devices if d['kind'] == 'cylinders' and a['id'] in d['ids'] and (not path or d['port'] == path)]
        if len(matches) != 1: raise ValueError('请选择唯一匹配的电缸总线；如刚插入请扫描')
        path = matches[0]['port']
        fresh = path not in self.buses
        port = self.buses.get(path) or Serial(path, self.baud)
        try:
            reply = port.cylinder(a['id']); mm = reply['position']*a['stroke']/2000
            low = -self.tolerance if a['minimum'] == 0 else a['minimum']
            if not low <= mm <= a['maximum']: raise ValueError(f'实际位置 {mm:.3f} mm 超出装配范围')
            self.buses[path] = port; self.axis_ports[i] = path
            a.update(online=True, armed=False, error='', comm_failures=0, position=mm, target=max(a['minimum'], mm), command=max(a['minimum'], mm),
                     last_seen=time.monotonic(), temperature=reply['temperature'])
        except Exception:
            if fresh: port.close()
            raise

    def heartbeat_time(self):
        client,stamp=self.heartbeat_seen
        return max(self.heartbeat,stamp) if client==self.owner else self.heartbeat

    def expire_control(self):
        if self.owner and time.monotonic()-self.heartbeat_time() > 1.5:
            self.experiment.cancel()
            self.stop('控制页面心跳超过 1.5 秒未更新')
            self.owner = None
            self.log('浏览器心跳失效：停止并撤销启用')

    def claim_control(self, client):
        if not client or len(client) > 100: raise ValueError('缺少控制会话')
        if self.owner and self.owner != client and time.monotonic()-self.heartbeat_time() < 1.5:
            raise ValueError('另一个浏览器正在控制；请先关闭其页面')
        if self.owner != client: self.stop(); self.owner = client
        self.heartbeat = time.monotonic()

    @staticmethod
    def is_side_autolocate(body):
        return (body.get('action'), body.get('device'), body.get('operation')) == (
            'device', 'camera.side_view', 'auto_locate')

    def auto_locate_side(self, body):
        client = str(body.get('client',''))
        with self.lock:
            if body.get('_script'):
                raise ValueError('侧视自动定位请在页面完成')
            self.claim_control(client)
            if self.experiment.process is not None:
                raise ValueError('请先停止实验再自动定位')
        candidate = self.side_view.roi_candidate()
        with self.lock:
            if (self.owner != client or time.monotonic()-self.heartbeat_time() > 1.5
                    or self.experiment.process is not None):
                raise ValueError('控制会话或实验状态已变化，请重新自动定位')
            return self.side_view.apply_auto_roi(candidate)

    def action(self, body):
        if body.get('action')=='heartbeat':
            client=str(body.get('client',''))
            accepted=bool(client and client==self.owner)
            if accepted:
                self.heartbeat_seen=(client,time.monotonic())
                self.heartbeat_count+=1
            return dict(ok=True,accepted=accepted,owner=self.owner)
        if self.is_side_autolocate(body):
            return self.auto_locate_side(body)
        with self.lock:
            op = body.get('action'); client = str(body.get('client', ''))
            if op == 'stop_all': self.experiment.cancel(); self.stop('全部停止按钮 / Esc'); return
            if op == 'automatic_stop': return  # Older pages must not stop on focus changes.
            self.claim_control(client)
            if op == 'script_stop': self.experiment.cancel(); self.stop(); return
            if op == 'script_clear':
                if self.experiment.process: raise ValueError('请先停止实验再清空输出')
                self.experiment.output.clear(); self.experiment.command=''; return
            if op == 'script_run': self.experiment.start(str(body.get('command','')),client); return
            if op == 'pretest_start' and not body.get('_script'):
                limit = body.get('max_advance_mm')
                if type(limit) not in (int, float) or not math.isfinite(limit) or not 0 < limit <= 48:
                    raise ValueError('请填写本次允许的三轴增量之和上限（mm）')
                camera = self.camera.snapshot()
                settings = camera['settings']
                if (not camera['online'] or camera['error'] or camera['recording'] or camera['frame_age'] is None
                        or camera['frame_age'] > 1 or camera['measurement_settings'] != settings
                        or body.get('settings') != settings or not (camera['measurement'] or {}).get('valid')):
                    raise ValueError('请先应用识别区域并查看新鲜、有效的相机预览')
                if shape_angle(camera['measurement']) is None:
                    raise ValueError('请先确认橙色近端与白色柔性段的三维中心线和角度有效')
                if any(a['moving'] for a in self.axes) or self.prop['armed'] or self.gamepad['active']:
                    raise ValueError('请先结束当前控制或运动')
                if not all(a['online'] and not a['error'] and time.monotonic()-a['last_seen']<1 for a in self.axes):
                    raise ValueError('请接入并检查三根电缸的反馈')
                self.experiment.start(f'python scripts/pretest.py --max-advance-mm {limit}',client,pretest=True)
                return
            if op in ('pretest_pause','pretest_resume') and not body.get('_script'):
                self.experiment.set_pause(op == 'pretest_pause'); return
            if op == 'pretest_progress' and body.get('_script'):
                return self.experiment.pretest_progress(body['progress'],begin_round=body.get('begin_round') is True)
            if self.experiment.process is not None and not body.get('_script'):
                raise ValueError('实验运行中，请先停止实验再手动控制')
            if op in ('axis_record_start', 'axis_record_stop'):
                if not body.get('_script'):
                    raise ValueError('双机记录仅由实验脚本控制')
                if op == 'axis_record_start':
                    side = self.side_view.snapshot()
                    if side['recording'] or side['finalizing']:
                        raise ValueError('侧视记录尚未结束')
                    result = self.camera.record_start(body.get('label', ''), body.get('group'))
                    try:
                        self.side_view.record_start(result['session_path'])
                    except Exception:
                        self.camera.stop_recording('侧视记录启动失败')
                        raise
                    return result
                side, overhead = self.side_view.snapshot(), self.camera.snapshot()
                self.side_view.stop_recording('本轮结束')
                self.camera.stop_recording('本轮结束')
                for stream in (side, overhead):
                    if stream['record_error']:
                        return {'error': stream['record_error']}
                    if not stream['recording'] or not stream['online'] or stream['frame_age'] is None or stream['frame_age'] > 1:
                        return {'error': stream['error'] or '双机记录提前中断'}
                return {'error': ''}
            if op == 'device':
                operation=body.get('operation')
                if body.get('_script') and operation in ('connect','disconnect','configure','auto_locate'):
                    raise ValueError('实验脚本使用已接入设备；连接与配置请在页面完成')
                return self.plugins.command(body.get('device'),operation,body.get('arguments',{}),
                                            dict(client=client,_script=bool(body.get('_script'))))
            if op == 'gamepad_stop': self.stop_gamepad(); return
            if op == 'gamepad_start':
                self.stop_gamepad()
                eligible=[i for i,a in enumerate(self.axes) if a['online'] and not a['error'] and time.monotonic()-a['last_seen']<1]
                for i in eligible: self.stop_axis(i)
                if self.prop['online']: self.stop_prop()
                eligible=[i for i in eligible if not self.axes[i]['error']]
                self.gamepad=dict(active=True,deadline=time.monotonic()+.6,axes=eligible,
                                  propulsion=self.prop['online'] and not self.prop['error'])
                for i in eligible: self.axes[i]['armed']=True
                self.stop_reason=''
                return
            if op == 'gamepad': self.gamepad_input(body); return
            if self.gamepad['active'] and op in ('motor','motor_stop','propulsion','move','move_group','jog','arm','arm_group'):
                raise ValueError('请先退出手柄控制')
            if op == 'motor_renew':
                now=time.monotonic()
                for i in range(4):
                    if self.prop['targets'][i] and now < self.prop['deadlines'][i]: self.prop['deadlines'][i]=now+.35
                return
            if op in ('motor', 'motor_stop'):
                i=body.get('motor')
                if type(i) is not int or i not in range(4): raise ValueError('电机编号必须为 1–4')
                if self.prop['backend'] not in ('pcb','sim'): raise ValueError('此设备不支持四电机独立控制')
                self.set_motor(i, 0 if op=='motor_stop' else body['value']); return
            if op == 'mode':
                if not self.testing or body.get('mode') not in ('simulation', 'hardware'): raise ValueError('控制台仅支持真实硬件')
                self.disconnect(); self.mode = body['mode']; self.devices = []; self.inventory = None; self.scan_enabled = False
                for a in self.axes: a.update(position=0., target=0., command=0., error='')
                self.log('切换至 '+self.mode+'；连接不会自动运动'); return
            if op == 'scan': self.scan_enabled = True; self.scan(); self.log('扫描完成；识别与连接独立'); return
            if op == 'arm_group':
                if not all(a['online'] and not a['error'] and time.monotonic()-a['last_seen']<1 for a in self.axes):
                    raise ValueError('需要三个电缸在线、无故障且反馈有效')
                for a in self.axes: a['armed']=True
                self.stop_reason=''
                return
            if 'axis' in body and (isinstance(body['axis'], bool) or body['axis'] not in (0, 1, 2)): raise ValueError('轴编号无效')
            if op == 'connect_axis': self.connect_axis(int(body['axis']), body.get('port')); self.log(f'电缸 {int(body["axis"])+1} 已接入'); return
            if op in ('disconnect_axis', 'stop_axis', 'arm', 'move', 'jog'):
                i = int(body['axis'])
                if not 0 <= i < 3: raise ValueError('轴编号无效')
                a = self.axes[i]
                if op in ('stop_axis', 'disconnect_axis'):
                    self.stop_axis(i)
                    if op == 'disconnect_axis':
                        a['online'] = False; path = self.axis_ports.pop(i, None)
                        if path and path not in self.axis_ports.values(): self.buses.pop(path).close()
                    return
                if not a['online'] or a['error']: raise ValueError('该轴未就绪；需要重新连接')
                if op == 'arm': a['armed'] = True; self.stop_reason=''; return
                if not a['armed']: raise ValueError('先启用此电缸')
                value = float(body['value'])
                target = a['position']+value if op == 'jog' else value
                if not math.isfinite(target) or not a['minimum'] <= target <= a['maximum']:
                    self.stop_axis(i); raise ValueError('目标超出装配限位；此轴已停止，需重新启用')
                a['target'] = target; a['moving'] = True; a['move_started'] = time.monotonic()
                self.log(f'电缸 {i+1} → {target:.3f} mm'); return
            if op == 'move_group':
                values = body['values']
                indices = body.get('axes', [0, 1, 2])
                if (not isinstance(indices, list) or not 2 <= len(indices) <= 3
                        or any(type(i) is not int or i not in range(3) for i in indices)
                        or len(set(indices)) != len(indices) or not isinstance(values, list) or len(values) != len(indices)):
                    raise ValueError('协同移动需要两个或三个不同的轴，并提供相同数量的目标')
                axes = [self.axes[i] for i in indices]
                try:
                    targets = [float(v) for v in values]
                    valid = all(a['online'] and a['armed'] and not a['error'] and a['minimum'] <= v <= a['maximum']
                                for a, v in zip(axes, targets))
                except (TypeError, ValueError, OverflowError):
                    valid = False
                if not valid:
                    for i in indices: self.stop_axis(i)
                    raise ValueError('协同移动要求所选轴已启用且目标在范围内；已撤销所选轴启用')
                started = time.monotonic()
                for a, target in zip(axes, targets):
                    a.update(target=target, moving=True, move_started=started)
                ids = ', '.join(str(a['id']) for a in axes)
                self.log(f'电缸 {ids} 的协同目标已更新（非硬件同步）'); return
            if op == 'connect_prop':
                if self.prop['online']: return
                if self.mode == 'simulation': self.prop.update(online=True, error='', backend='sim', speed=0); return
                matches = [d for d in self.devices if d['kind'] in ('pcb', 'nano') and (not body.get('port') or d['port'] == body['port'])]
                if len(matches) != 1: raise ValueError('请选择唯一匹配的推进器')
                if self.prop_serial: self.prop_serial.close(); self.prop_serial = None
                d = matches[0]; port = Serial(d['port'])
                try:
                    if d['kind'] == 'nano': time.sleep(2); port.line('?', 'V')
                    else: port.line('INFO', 'INFO PCB1_S3 ')
                    port.line('V0', 'OK V0')
                    if d['kind'] == 'pcb': port.line('A0', 'OK A0')
                except Exception: port.close(); raise
                self.prop_serial = port; self.prop.update(online=True, error='', backend=d['kind'], port=d['port']); return
            if op == 'stop_prop': self.stop_prop(); return
            if op == 'disconnect_prop':
                self.stop_prop()
                if self.prop_serial: self.prop_serial.close(); self.prop_serial = None
                self.prop['online'] = False; return
            if op == 'propulsion':
                self.set_motor(None, body['value']); return
            raise ValueError('未知操作')

    def stop_gamepad(self):
        if self.gamepad['active']:
            for i in self.gamepad['axes']: self.stop_axis(i)
            if self.gamepad['propulsion']: self.stop_prop()
        self.gamepad['active']=False
        self.gamepad_moving.clear()

    def gamepad_input(self, body):
        if not self.gamepad['active']: raise ValueError('手柄控制未启用，请回中后重新启用')
        if time.monotonic()>self.gamepad['deadline']:
            self.stop_gamepad()
            raise ValueError('手柄输入超时，请重新启用')
        values=body.get('values')
        speed=body.get('speed',400)
        if (not isinstance(values,list) or len(values)!=4 or
            any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in values) or
            type(speed) not in (int,float) or not math.isfinite(speed) or not 0<=speed<=1600):
            self.stop_gamepad()
            raise ValueError('手柄输入无效')
        self.gamepad['deadline']=time.monotonic()+.6
        for i in self.gamepad['axes']:
            a=self.axes[i]; v=values[i+1]
            if not a['online'] or not a['armed'] or a['error'] or time.monotonic()-a['last_seen']>=1:
                continue
            if abs(v)<.001:
                if i in self.gamepad_moving:
                    self.stop_axis(i)
                    if not a['error']: a['armed']=True
                    self.gamepad_moving.discard(i)
            else:
                a.update(target=max(a['minimum'],min(a['maximum'],a['position']+v*a['speed']*.2)),
                         moving=True,move_started=time.monotonic())
                self.gamepad_moving.add(i)
        if self.gamepad['propulsion'] and self.prop['online'] and not self.prop['error']:
            self.set_motor(None,round(values[0]*speed))

    def set_motor(self, index, value):
        value=float(value)
        if not math.isfinite(value) or not value.is_integer() or abs(value)>1600 or not self.prop['online'] or self.prop['error']:
            raise ValueError('推进器未就绪或速度超限（整数 ±1600 step/s）')
        if self.prop_serial and self.prop['backend']=='pcb' and not self.prop['armed'] and value:
            try: self.prop_serial.line('A1','OK A1')
            except Exception: self.stop_prop(); raise
        indices=range(4) if index is None else [index]
        for i in indices:
            self.prop['targets'][i]=int(value); self.prop['deadlines'][i]=time.monotonic()+.35
        self.prop['speed']=self.prop['targets'][0] if len(set(self.prop['targets']))==1 else None
        self.prop['armed']=self.prop['armed'] or bool(value)
        try: self.write_motors()
        except Exception: self.stop_prop(); raise

    def write_motors(self):
        if not self.prop_serial: return
        targets=self.prop['targets']
        if self.prop['backend']=='pcb':
            if self.prop['armed']: self.prop_serial.line('W'+','.join(map(str,targets)), 'OK AXES')
        else: self.prop_serial.line('V'+str(targets[0]),'OK V'+str(targets[0]))

    def tick(self, dt):
        with self.lock:
            now = time.monotonic()
            self.expire_control()
            if self.gamepad['active'] and now>self.gamepad['deadline']:
                self.stop_gamepad()
                self.stop_reason='手柄输入超过 0.6 秒未更新；请重新启用'
            for i, a in enumerate(self.axes):
                if not a['online']: continue
                querying=False
                try:
                    if self.mode == 'hardware':
                        port = self.buses[self.axis_ports[i]]; querying=True
                        reply = port.cylinder(a['id']); querying=False
                        a['position'] = reply['position']*a['stroke']/2000; a['temperature'] = reply['temperature']
                        if not (-self.tolerance if a['minimum']==0 else a['minimum']) <= a['position'] <= a['maximum']: raise ValueError('反馈超出限位')
                    if a['comm_failures']:
                        a['comm_failures']=0; a['comm_recoveries']+=1; a['error']=''
                    a['last_seen'] = time.monotonic()
                    self.expire_control()
                    if a['armed'] and a['moving']:
                        if time.monotonic()-a['move_started'] > 10+4*a['stroke']/a['speed']: raise TimeoutError('运动到位超时')
                        goal_code = max(math.ceil(a['minimum']/a['stroke']*2000), min(math.floor(a['maximum']/a['stroke']*2000), round(a['target']/a['stroke']*2000)))
                        goal = goal_code*a['stroke']/2000
                        step = a['speed']*max(0, min(dt, .2))
                        next_pos = a['command'] + max(-step, min(step, goal-a['command']))
                        measured = max(a['minimum'], min(a['maximum'], a['position']))
                        lead = a['speed']*.2+a['stroke']/2000
                        a['command'] = max(a['minimum'], min(a['maximum'], max(measured-lead, min(measured+lead, next_pos))))
                        code = max(math.ceil(a['minimum']/a['stroke']*2000), min(math.floor(a['maximum']/a['stroke']*2000), round(a['command']/a['stroke']*2000)))
                        if self.mode == 'simulation': a['position'] = code*a['stroke']/2000
                        elif code != reply['target']:
                            a['owned'] = True
                            ack = port.cylinder(a['id'], 0x25, (2, 0, 0, math.floor(a['speed']/a['stroke']*2000), code))
                            if ack['target'] != code: raise RuntimeError('电缸未接受目标')
                        tolerance = max(a['stroke']/4000, self.tolerance if goal == 0 else 0)
                        if abs(a['position']-goal) <= tolerance:
                            a['moving'] = False; self.log(f'电缸 {i+1} 到位（位置反馈）')
                    elif not a['moving']: a['command'] = max(a['minimum'], min(a['maximum'], a['position']))
                except Exception as e:
                    self.stop_reason=f'电缸 {a["id"]} 异常，同总线停机: {e}'
                    a['last_comm_error']=str(e)
                    path = self.axis_ports.get(i)
                    affected=[j for j in range(3) if j==i or (path and self.axis_ports.get(j)==path)]
                    for j in affected: self.stop_axis(j)
                    stop_failed=any(self.axes[j]['error'].startswith('停止未确认:') for j in affected)
                    if querying and isinstance(e, TimeoutError) and not stop_failed:
                        a['comm_failures']+=1
                        a.update(armed=False,moving=False)
                        if a['comm_failures']<3:
                            a['error']=f"通信重试 {a['comm_failures']}/3；已停止，恢复后需重新启用"
                        else:
                            a.update(online=False,error='连续 3 次查询失败: '+str(e))
                    else:
                        a.update(online=False,armed=False,moving=False,
                                 error=a['error'] if a['error'].startswith('停止未确认:') else str(e))
                    self.log(f'电缸 {i+1}: {e}')
            if self.prop['online']:
                try:
                    self.expire_control()
                    now=time.monotonic()
                    for i in range(4):
                        if now > self.prop['deadlines'][i]: self.prop['targets'][i]=0
                    self.prop['speed']=self.prop['targets'][0] if len(set(self.prop['targets']))==1 else None
                    if self.prop['armed']:
                        if any(self.prop['targets']): self.write_motors()
                        else: self.stop_prop()
                    if self.prop_serial:
                        if self.prop['backend']=='pcb':
                            status = json.loads(self.prop_serial.line('STATUS', 'STATUS ')[7:])
                            speeds, currents = status.get('speed'), status.get('current_ma')
                            if not isinstance(speeds, list) or not isinstance(currents, list) or len(speeds)!=4 or len(currents)!=4:
                                raise RuntimeError('PCB 四电机状态不完整')
                            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in speeds+currents):
                                raise RuntimeError('PCB 电机反馈无效')
                            self.prop['motors'] = [dict(id=i+1, speed=speeds[i], current_ma=currents[i]) for i in range(4)]
                            self.prop['telemetry_time'] = now
                            if status.get('fault') or status.get('latched'): raise RuntimeError('PCB 报告故障或锁止')
                        else:
                            self.prop_serial.line('?', 'V')
                            self.prop['motors'] = []
                except Exception as e:
                    self.stop_prop(); self.prop.update(online=False, error=str(e)); self.log('推进器: '+str(e))

            self._telemetry_cache=dict(valid=True,host_monotonic=time.monotonic(),axes=copy.deepcopy(self.axes),propulsion=copy.deepcopy(self.prop))

    def loop(self):
        previous = time.monotonic()
        while self.running:
            now = time.monotonic()
            self.tick(now-previous); previous = now
            with self.lock:
                if self.mode == 'hardware' and self.scan_enabled and not any(a['armed'] for a in self.axes) and not self.prop['armed'] and now-self.last_scan > 3:
                    if self.ports() != self.inventory:
                        try: self.scan(); self.log('USB 设备列表变化，已重新识别；不会自动连接或恢复运动')
                        except Exception as e: self.log('扫描失败: '+str(e))
                    self.last_scan = time.monotonic()
            time.sleep(.06)

    def close(self):
        self.running = False
        with self.lock: self.experiment.cancel(); self.disconnect()
        self.side_view.close()
        self.camera.close()

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def send(self, code, data, kind='application/json', headers=None):
        raw = json.dumps(data, ensure_ascii=False).encode() if kind=='application/json' else data
        try:
            self.send_response(code); self.send_header('Content-Type', kind+('; charset=utf-8' if kind.startswith('text/') or kind=='application/json' else ''))
            self.send_header('Content-Length', str(len(raw))); self.send_header('Cache-Control', 'no-store')
            for name, value in (headers or {}).items(): self.send_header(name, value)
            self.send_header('X-Content-Type-Options', 'nosniff'); self.end_headers(); self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            # A closing browser cannot receive the response; the control lease still expires.
            self.close_connection = True
    def trusted(self):
        hosts = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
        return self.headers.get('Host') in hosts and self.headers.get('Origin', 'http://'+self.headers.get('Host','')) in {'http://'+h for h in hosts}
    def do_GET(self):
        if not self.trusted(): self.send(403, {'error':'仅允许本机同源请求'}); return
        if self.path == '/api/state': self.send(200, self.server.engine.state()); return
        if self.path in ('/api/camera/d405/devices', '/api/camera/side-view/devices'):
            engine = self.server.engine
            camera = engine.camera if self.path == '/api/camera/d405/devices' else engine.side_view
            try:
                self.send(200, camera.discover())
            except (ValueError, RuntimeError) as error:
                self.send(503, {'error': str(error)})
            return
        if self.path in ('/api/camera/preview', '/api/camera/side-view/preview'):
            engine = self.server.engine
            camera = engine.side_view if self.path.endswith('/side-view/preview') else engine.camera
            preview=camera.preview(with_settings=True)
            if preview is None:self.send(404, {'error':'相机没有新鲜图像'})
            else:
                jpeg, settings = preview
                self.send(200,jpeg,'image/jpeg',{'X-Camera-Settings':json.dumps(settings)})
            return
        files = {'/':('index.html','text/html'), '/app.js':('app.js','text/javascript'),
                 '/connection-worker.js':('connection-worker.js','text/javascript'), '/scene.js':('scene.js','text/javascript'), '/camera.js':('camera.js','text/javascript'), '/side-camera.js':('side-camera.js','text/javascript'), '/camera-shape.js':('camera-shape.js','text/javascript'), '/gamepad.js':('gamepad.js','text/javascript'), '/style.css':('style.css','text/css')}
        vendors = ('three.module.js', 'GLTFLoader.js', 'OrbitControls.js', 'BufferGeometryUtils.js', 'RoomEnvironment.js')
        for name in vendors: files['/vendor/'+name] = ('vendor/'+name, 'text/javascript')
        models = {'/models/cylinders.glb':'Triple_Cylinder_Syringes_V3.glb', '/models/advancer.glb':'Advancer_Assembly.glb', '/models/d405-stand.glb':'D405_Stand_H300.glb'}
        if self.path in models:
            path = ROOT.parent.parent/'3d model'/models[self.path]
            if not path.is_file(): self.send(404, {'error':'模型文件不存在'}); return
            self.send(200, path.read_bytes(), 'model/gltf-binary'); return
        if self.path not in files: self.send(404, {'error':'not found'}); return
        name, kind = files[self.path]; self.send(200, (ROOT/'static'/name).read_bytes(), kind)
    def do_POST(self):
        if not self.trusted() or self.headers.get('Content-Type') != 'application/json': self.send(403, {'error':'仅允许本机JSON请求'}); return
        try:
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= 4096 or self.path not in ('/api/action','/api/script'): raise ValueError('请求无效')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body,dict): raise ValueError('请求必须是 JSON 对象')
            engine=self.server.engine
            body.pop('_script',None)
            if self.path=='/api/action' and body.get('action')=='heartbeat':
                result=engine.action(body)
                if body.get('renew_motors'):
                    with engine.lock:
                        if body.get('client')==engine.owner and engine.experiment.process is None:
                            engine.action(dict(action='motor_renew',client=body['client']))
                if body.get('wait') is True: time.sleep(.1)
                self.send(200, result); return
            if self.path == '/api/action' and engine.is_side_autolocate(body):
                self.send(200, {'ok':True,'result':engine.action(body)})
                return
            with engine.lock:
                if self.path == '/api/script':
                    import secrets
                    token=self.headers.get('X-Experiment-Token','')
                    if not engine.experiment.token or not secrets.compare_digest(token,engine.experiment.token): raise ValueError('实验已结束或凭证无效')
                    if time.monotonic()-engine.heartbeat_time() > 1.5: raise ValueError('浏览器已失联')
                    if body.get('action') not in ('motor','motor_stop','motor_renew','propulsion','stop_prop','arm','move','move_group','jog','stop_axis','stop_all','device','pretest_progress','axis_record_start','axis_record_stop'):
                        raise ValueError('脚本不支持此操作')
                    body.update(client=engine.experiment.client,_script=True)
                    last_heartbeat=engine.heartbeat
                    try: result=engine.action(body)
                    finally: engine.heartbeat=last_heartbeat
                else: result=engine.action(body)
            self.send(200, {'ok':True,'result':result})
        except Exception as e: self.send(400, {'error':str(e)})

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--config', type=Path, default=ROOT.parent/'system_control'/'cylinders.conf')
    args = parser.parse_args()
    engine = Engine(args.config)
    http = ThreadingHTTPServer(('127.0.0.1', args.port), Handler); http.engine = engine; engine.experiment.port=http.server_port
    thread = threading.Thread(target=engine.loop, daemon=True); thread.start()
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    print(f'Local control: http://127.0.0.1:{args.port} — hardware; disconnected; no hardware opened', flush=True)
    try: http.serve_forever()
    except KeyboardInterrupt: pass
    finally: engine.close(); http.server_close(); thread.join(timeout=2)
if __name__ == '__main__': main()
