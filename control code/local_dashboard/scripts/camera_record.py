"""Record an already connected D405; never enable or move actuators."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from experiment_api import Robot
from sampling import require_camera


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--label',default='reference')
    args=p.parse_args()
    if not 0<args.seconds<=600:p.error('seconds 需要在 0–600 之间')
    import time
    with Robot() as robot:
        require_camera(robot.state())
        result=robot.record_start(args.label);print('记录目录：'+result['session_path'],flush=True)
        try:
            end=time.monotonic()+args.seconds
            while time.monotonic()<end:
                require_camera(robot.state(),recording=True)
                robot.wait(.1)
        finally:
            try:robot.record_stop()
            except RuntimeError:pass
        print('记录完成；未发送运动指令',flush=True)

if __name__=='__main__':main()
