"""Validate a position CSV, or explicitly execute it with D405 recording and bounded moves."""
import argparse
import json
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from experiment_api import Robot
from sampling import read_plan, require_camera, axes_by_id, wait_stable


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',required=True,help='CSV: q1_mm,q2_mm,q3_mm,hold_s')
    p.add_argument('--limits',required=True,nargs=6,type=float,metavar=('LO1','HI1','LO2','HI2','LO3','HI3'))
    p.add_argument('--max-total-mm',required=True,type=float,help='本次已确认的三轴绝对位置之和上限')
    p.add_argument('--execute',action='store_true',help='实际启用并移动电缸；省略时仅校验文件')
    p.add_argument('--max-step-mm',type=float,default=.1)
    p.add_argument('--timeout',type=float,default=30)
    p.add_argument('--stable-window',type=float,default=2)
    p.add_argument('--angle-tolerance',type=float,default=1,help='表面角稳定范围（度），需按实测噪声调整')
    p.add_argument('--position-tolerance',type=float,default=.05)
    p.add_argument('--shape-tolerance',type=float,default=.5,help='稳定窗内表面曲线点位变化上限 mm，需按实测噪声调整')
    args=p.parse_args()
    values=[args.max_step_mm,args.timeout,args.stable_window,args.angle_tolerance,args.position_tolerance,args.shape_tolerance]
    if not all(math.isfinite(v) and v>0 for v in values) or args.max_step_mm>.5 or args.stable_window>=args.timeout:
        p.error('步长须在 0–0.5 mm 内，时间和容差须为正且稳定窗小于超时')
    rows,pairs=read_plan(args.plan,args.limits,args.max_total_mm)
    print(json.dumps(dict(points=len(rows),limits=pairs,max_total_mm=args.max_total_mm,execute=args.execute),ensure_ascii=False),flush=True)
    if not args.execute:return
    with Robot() as robot:
        s=robot.state();require_camera(s,shape=True);axes=axes_by_id(s)
        for a,(lo,hi) in zip(axes,pairs):
            if not a['online'] or a['error'] or a['age'] is None or a['age']>1:raise RuntimeError('请先接入三轴并确认反馈')
            feedback_low=-s['tolerance'] if lo==a['minimum']==0 else lo
            if lo<a['minimum'] or hi>a['maximum'] or not feedback_low<=a['position']<=hi:raise RuntimeError('实验范围或起点超出装配限位')
        current=[max(lo,min(hi,a['position'])) for a,(lo,hi) in zip(axes,pairs)]
        if sum(current)>args.max_total_mm:raise RuntimeError('当前三轴总位移超出实验上限')
        try:
            result=robot.record_start('position_sampling');print(result['session_path'],flush=True)
            (Path(result['session_path'])/'experiment_plan.json').write_text(json.dumps(dict(arguments=vars(args),targets=rows),ensure_ascii=False,indent=2))
            robot.annotate(kind='plan',total_points=len(rows),limits=pairs,max_total_mm=args.max_total_mm)
            for i in (1,2,3):robot.device('cylinder.'+str(i),'arm')
            for index,row in enumerate(rows):
                target=row['q'];delta=max(abs(a-b) for a,b in zip(current,target));steps=max(1,math.ceil(delta/args.max_step_mm))
                start=current[:]
                for step in range(1,steps+1):
                    intermediate=[a+(b-a)*step/steps for a,b in zip(start,target)]
                    require_camera(robot.state(),recording=True,shape=True)
                    # Decreases first keeps the sum bound valid during sequential writes.
                    order=sorted(range(3),key=lambda i:intermediate[i]-current[i])
                    pending=current[:]
                    for i in order:
                        if abs(intermediate[i]-pending[i])<1e-9:continue
                        robot.device('cylinder.'+str(i+1),'move',mm=intermediate[i])
                        pending[i]=intermediate[i]
                        wait_stable(robot,pending,args.timeout,.3,args.angle_tolerance,args.position_tolerance,args.shape_tolerance)
                    current=intermediate
                robot.annotate(kind='target',sample=index,target=target)
                stable=wait_stable(robot,target,args.timeout,args.stable_window,args.angle_tolerance,args.position_tolerance,args.shape_tolerance)
                reference=stable['camera']['measurement']['surface_curve_camera_mm']
                robot.annotate(kind='steady_start',sample=index,target=target)
                import time
                end=time.monotonic()+row['hold_s']
                while time.monotonic()<end:
                    state=robot.state();require_camera(state,recording=True,shape=True)
                    if any(a['error'] or not a['armed'] or not a['online'] or a['age'] is None or a['age']>1 or abs(a['position']-q)>args.position_tolerance for a,q in zip(axes_by_id(state),target)):raise RuntimeError('保持阶段电缸失效或位置漂移')
                    curve=state['camera']['measurement']['surface_curve_camera_mm']
                    if len(curve)!=21 or any(p is None for p in curve) or max(math.dist(a,b) for a,b in zip(curve,reference))>args.shape_tolerance:
                        robot.annotate(kind='unstable',sample=index)
                        raise RuntimeError('保持阶段形状漂移；此样本未完成')
                    robot.wait(.1)
                robot.annotate(kind='steady_end',sample=index)
                print(f'完成样本 {index+1}/{len(rows)}: {target}',flush=True)
        finally:
            for i in (1,2,3):
                try:robot.device('cylinder.'+str(i),'stop')
                except RuntimeError:pass
            try:robot.record_stop()
            except RuntimeError:pass

if __name__=='__main__':main()
