"""Run one already connected PCB motor for a bounded interval."""
import argparse
from experiment_api import Robot
parser=argparse.ArgumentParser()
parser.add_argument('--motor',type=int,choices=range(1,5),required=True)
parser.add_argument('--speed',type=int,required=True,help='signed step/s; positive forward, negative reverse')
parser.add_argument('--seconds',type=float,required=True)
args=parser.parse_args()
if not 0<args.seconds<=60: parser.error('seconds must be in (0,60]')
if not -1600<=args.speed<=1600: parser.error('speed must be within ±1600')
robot=Robot()
try:
    print(f'M{args.motor}: {args.speed} step/s, {args.seconds} s',flush=True)
    robot.motor(args.motor,args.speed)
    robot.wait(args.seconds)
finally:
    robot.stop_motor(args.motor)
print('完成')
