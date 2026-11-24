"""Validation and observation helpers shared by experiment scripts and offline tests."""
import csv
import math
import time
from collections import deque


def read_plan(path, limits, max_total):
    if len(limits)!=6 or not all(math.isfinite(v) for v in limits):raise ValueError('需要三轴各自的有限上下限')
    pairs=list(zip(limits[::2],limits[1::2]))
    if any(not 0<=lo<hi<=16 for lo,hi in pairs):raise ValueError('每轴实验上下限须在 0–16 mm 内且下限小于上限')
    if not math.isfinite(max_total) or max_total<=0:raise ValueError('总位移上限须为正数')
    with open(path,newline='') as f:
        reader=csv.DictReader(f)
        if reader.fieldnames!=['q1_mm','q2_mm','q3_mm','hold_s']:raise ValueError('CSV 表头必须为 q1_mm,q2_mm,q3_mm,hold_s')
        rows=[]
        for i,row in enumerate(reader,2):
            q=[float(row[k]) for k in ('q1_mm','q2_mm','q3_mm')];hold=float(row['hold_s'])
            if not all(math.isfinite(v) for v in q+[hold]) or not 0<hold<=60:raise ValueError(f'第 {i} 行数值或保持时间无效')
            if any(not lo<=v<=hi for v,(lo,hi) in zip(q,pairs)) or sum(q)>max_total:raise ValueError(f'第 {i} 行超出实验限制')
            rows.append(dict(q=q,hold_s=hold))
            if len(rows)>1000:raise ValueError('每个计划最多 1000 个目标')
    if not rows:raise ValueError('计划为空')
    return rows,pairs


def require_stream(camera, *, recording=False, name='相机'):
    if not camera or not camera['online'] or camera['error'] or camera['frame_age'] is None or camera['frame_age'] > 1:
        raise RuntimeError((camera or {}).get('error') or f'{name}图像失效')
    if recording and (not camera['recording'] or camera['record_error']):
        raise RuntimeError(camera['record_error'] or f'{name}记录已中断')
    return camera


def require_camera(state, recording=False, shape=False):
    c = require_stream(state['camera'], recording=recording)
    m=c.get('measurement') or {}
    if shape and (not m.get('valid') or m.get('depth_fraction',0)<.8 or m.get('surface_theta_deg') is None):
        raise RuntimeError('柔性段轮廓或深度不足；停止采样运动')
    return c


def axes_by_id(state):
    axes={a['id']:a for a in state['axes']}
    if set(axes)!={1,2,3}:raise RuntimeError('采样脚本要求设备 ID 1、2、3')
    return [axes[i] for i in (1,2,3)]


def positions_at_target(axes, targets, tolerance):
    for axis, target in zip(axes,targets):
        if not abs(axis['position']-target)<=tolerance:
            return False
        if axis['moving']:
            # The driver can keep its motion bit set after settling a few counts from the goal.
            half_count = axis['stroke']/4000 + 1e-9
            if not (abs(axis['target']-target)<=half_count and abs(axis['command']-target)<=half_count):
                return False
    return True


class ArrivalWindow:
    """Require distinct encoder samples and at most three counts of travel per axis."""

    def __init__(self, targets, tolerance, seconds=.6):
        self.targets = targets
        self.tolerance = tolerance
        self.seconds = seconds
        self.samples = deque()
        self.last_seen = None

    def update(self, axes):
        stamps = tuple(a['last_seen'] for a in axes)
        if not positions_at_target(axes,self.targets,self.tolerance) or min(stamps)<=0:
            self.samples.clear()
            self.last_seen = None
            return False
        if self.last_seen and any(s<previous or s-previous>self.seconds for s,previous in zip(stamps,self.last_seen)):
            self.samples.clear()
            self.last_seen = None
        if not self.last_seen or all(s>previous for s,previous in zip(stamps,self.last_seen)):
            self.last_seen = stamps
            self.samples.append((min(stamps), tuple(a['position'] for a in axes)))
        latest = self.samples[-1][0]
        while len(self.samples)>3 and latest-self.samples[1][0]>=self.seconds:
            self.samples.popleft()
        if len(self.samples)<3 or latest-self.samples[0][0]<self.seconds:
            return False
        columns = zip(*(positions for _,positions in self.samples))
        return all(max(values)-min(values)<=3*axis['stroke']/2000+1e-9 for axis,values in zip(axes,columns))


def wait_stable(robot, target, timeout, window, angle_tolerance, position_tolerance, shape_tolerance=.5):
    recent=deque();deadline=time.monotonic()+timeout;last_frame=None
    while time.monotonic()<deadline:
        s=robot.state();c=require_camera(s,recording=True,shape=True)
        axes=axes_by_id(s)
        if any(not a['online'] or a['error'] or not a['armed'] or a['age'] is None or a['age']>1 for a in axes):
            raise RuntimeError('电缸反馈失效或已停机')
        arrived=all(not a['moving'] and abs(a['position']-q)<=position_tolerance for a,q in zip(axes,target))
        now=time.monotonic()
        if not arrived:recent.clear()
        elif c['frame_number']!=last_frame:
            curve=c['measurement'].get('surface_curve_camera_mm',[])
            if len(curve)!=21 or any(p is None or len(p)!=3 or not all(math.isfinite(v) for v in p) for p in curve):
                raise RuntimeError('完整三维表面曲线缺失')
            recent.append((now,c['measurement']['surface_theta_deg'],curve))
            while len(recent)>2 and recent[1][0]<now-window:recent.popleft()
            last_frame=c['frame_number']
            if len(recent)>=5 and recent[-1][0]-recent[0][0]>=window:
                values=[item[1] for item in recent]
                reference=recent[0][2]
                displacement=max(math.dist(p,q) for item in recent for p,q in zip(reference,item[2]))
                if max(values)-min(values)<=angle_tolerance and displacement<=shape_tolerance:return s
        robot.wait(.1)
    raise TimeoutError('位置或视觉角度未在指定时间内稳定')
