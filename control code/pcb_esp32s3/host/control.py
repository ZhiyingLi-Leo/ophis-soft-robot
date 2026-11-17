#!/usr/bin/env python3
"""新PCB统一终端、键盘、手柄和状态采集入口；无端口时须显式--dry-run。"""
from __future__ import annotations
import argparse,asyncio,csv,json,math,sys,time,threading,queue
from datetime import datetime,timezone
from client import Client,USBLink,BLELink,DryLink

def axis_speed(axis,maximum,deadzone):
    if not math.isfinite(axis):raise ValueError("摇杆值不是有限数")
    axis=max(-1.,min(1.,axis))
    return 0 if abs(axis)<=deadzone else round(math.copysign((abs(axis)-deadzone)/(1-deadzone)*maximum,axis))

class Recorder:
    def __init__(self,path):
        self.file=open(path,'w',newline='',encoding='utf-8') if path else None
        if self.file:self.csv=csv.writer(self.file);self.csv.writerow(['utc','host_monotonic_s','board_uptime_ms','armed','owner','external_selected','fault_mask','latched','m1_command_sps','m2_command_sps','m3_command_sps','m4_command_sps','event'])
    def write(self,s):
        if self.file:
            self.csv.writerow([datetime.now(timezone.utc).isoformat(),time.monotonic(),s.get('uptime_ms'),s.get('armed'),s.get('owner'),s.get('external'),s.get('fault'),s.get('latched'),*s.get('speed',[0]*4),s.get('event')]);self.file.flush()
    def close(self):
        if self.file:self.file.close()

async def collect(client,recorder):
    while True:
        s=await client.status();recorder.write(s)
        if s.get('fault') or s.get('latched'):raise RuntimeError(f"板端故障：{s}")
        await asyncio.sleep(.25)

async def interactive(client):
    print('终端命令：A1解锁；V400同步前进；M1,400单轴；S停止并保持；A0释放；STATUS；quit退出。')
    incoming=queue.Queue()
    def read_input():
        while True:
            line=sys.stdin.readline();incoming.put(line)
            if not line:return
    threading.Thread(target=read_input,daemon=True).start()
    print('PCB> ',end='',flush=True)
    while True:
        try:raw=incoming.get_nowait()
        except queue.Empty:
            if client.failure:raise RuntimeError(str(client.failure))
            await asyncio.sleep(.05);continue
        if not raw:return
        line=raw.strip()
        if line.lower() in ('quit','exit'):return
        if not line:continue
        try:print(await client.command(line))
        except Exception as e:print(e)
        print('PCB> ',end='',flush=True)

async def teleop(client,args):
    import pygame
    pygame.init();pygame.joystick.init()
    screen=pygame.display.set_mode((620,150))
    pygame.display.set_caption('Propulsion PCB — keep this window focused')
    font=pygame.font.Font(None,25)
    joy=None
    if args.mode=='gamepad':
        if pygame.joystick.get_count()<1:raise RuntimeError('未检测到手柄；先连接手柄，或使用keyboard模式')
        joy=pygame.joystick.Joystick(0);joy.init()
        if args.axis_index>=joy.get_numaxes() or args.stop_button>=joy.get_numbuttons():raise RuntimeError('手柄轴/急停键映射越界；请先dry-run检查映射')
    await client.arm();current=0;last_sent=None;last=time.monotonic()
    print('W/↑前进，S/↓回退；松键停止；空格停止；Esc退出。手柄默认axis[3]、stop button[6]。')
    try:
        while True:
            now=time.monotonic();dt=min(now-last,.1);last=now;stop=False;quit_now=False
            for e in pygame.event.get():
                if e.type==pygame.QUIT or (e.type==pygame.KEYDOWN and e.key==pygame.K_ESCAPE):quit_now=True
                if e.type==getattr(pygame,'JOYDEVICEREMOVED',-1):raise RuntimeError('手柄已断开')
            keys=pygame.key.get_pressed();focused=pygame.key.get_focused()
            if joy:
                if not joy.get_init():raise RuntimeError('手柄失效')
                value=joy.get_axis(args.axis_index)*args.axis_sign;stop=bool(joy.get_button(args.stop_button))
            else:value=float(keys[pygame.K_w] or keys[pygame.K_UP])-float(keys[pygame.K_s] or keys[pygame.K_DOWN])
            target=axis_speed(value,args.max_speed,args.deadzone)
            stop=stop or keys[pygame.K_SPACE] or not focused or quit_now
            if stop or target==0:current=0
            else:
                delta=max(1,round(args.acceleration*dt));current=max(current-delta,min(current+delta,target))
            if current!=last_sent:await client.velocity(current);last_sent=current
            if client.failure:raise RuntimeError(str(client.failure))
            screen.fill((20,36,48));screen.blit(font.render(f'Command: {current} steps/s | Space: stop | Esc: exit',True,(230,240,245)),(20,35));pygame.display.flip()
            if quit_now:return
            await asyncio.sleep(.02)
    finally:
        try:await client.command('S')
        finally:pygame.quit()

async def run(args):
    if args.scan:
        from bleak import BleakScanner
        for d in await BleakScanner.discover(timeout=5):print(d.address,d.name)
        return
    link=DryLink() if args.dry_run else BLELink(args.ble or None) if args.ble is not None else USBLink(args.port)
    client=Client(link);recorder=Recorder(args.log);logger=None
    print(await client.open())
    try:
        if args.mode=='status':
            s=await client.status();recorder.write(s);print(json.dumps(s,ensure_ascii=False,indent=2));return
        if args.mode=='command':print(await client.command(args.line));return
        if args.log and args.mode!='jog':logger=asyncio.create_task(collect(client,recorder))
        if args.mode=='jog':
            await client.arm();await client.velocity(args.speed,args.motor)
            # Poll status even without logging; a fault ends the jog immediately.
            end=time.monotonic()+args.duration
            while time.monotonic()<end:
                if client.failure:raise RuntimeError(str(client.failure))
                if logger and logger.done():await logger
                s=await client.status();recorder.write(s)
                if not s.get('armed') or s.get('fault') or s.get('latched'):raise RuntimeError(f'板端拒绝继续：{s}')
                await asyncio.sleep(.1)
            print(await client.command('S'))
        else:
            task=asyncio.create_task(interactive(client) if args.mode=='terminal' else teleop(client,args))
            if logger:
                done,_=await asyncio.wait([task,logger],return_when=asyncio.FIRST_COMPLETED)
                if logger in done:
                    task.cancel()
                    try:await task
                    except asyncio.CancelledError:pass
                    await logger
                else:await task
            else:await task
    finally:
        if logger:
            logger.cancel()
            try:await logger
            except (asyncio.CancelledError,Exception):pass
        try:await client.close()
        finally:recorder.close()

def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    t=p.add_mutually_exclusive_group(required=True);t.add_argument('--port');t.add_argument('--ble',nargs='?',const='');t.add_argument('--dry-run',action='store_true');t.add_argument('--scan',action='store_true')
    p.add_argument('--log',help='CSV采集路径，记录指令与板端状态，不是编码器实测速度')
    sub=p.add_subparsers(dest='mode')
    sub.add_parser('status');sub.add_parser('terminal')
    c=sub.add_parser('command');c.add_argument('line')
    j=sub.add_parser('jog');j.add_argument('--speed',type=int,default=400);j.add_argument('--duration',type=float,default=.5);j.add_argument('--motor',type=int,choices=[1,2,3,4])
    for mode in ['keyboard','gamepad']:
        a=sub.add_parser(mode);a.add_argument('--max-speed',type=int,default=3200);a.add_argument('--acceleration',type=float,default=3200);a.add_argument('--deadzone',type=float,default=.12);a.add_argument('--axis-index',type=int,default=3);a.add_argument('--axis-sign',type=int,choices=[-1,1],default=1);a.add_argument('--stop-button',type=int,default=6)
    a=p.parse_args(argv)
    if not a.scan and not a.mode:p.error('请选择status、terminal、jog、keyboard、gamepad或command')
    if a.mode=='jog' and (not math.isfinite(a.duration) or not 0<a.duration<=30 or not 0<abs(a.speed)<=16000):p.error('点动速度须1..16000，持续时间须0..30秒')
    if a.mode in ['keyboard','gamepad'] and (not 0<a.max_speed<=16000 or not math.isfinite(a.acceleration) or a.acceleration<=0 or not 0<=a.deadzone<1 or a.axis_index<0 or a.stop_button<0):p.error('速度、加速度、死区或手柄映射无效')
    return a

if __name__=='__main__':
    try:asyncio.run(run(parse_args()))
    except (KeyboardInterrupt,EOFError):pass
    except Exception as e:print(f'停止：{e}',file=sys.stderr);sys.exit(1)
