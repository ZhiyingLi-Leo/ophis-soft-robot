"""POSIX pseudo-terminal integration against compiled C++ controller logic, no hardware."""
import asyncio,json,os,subprocess,sys,time,tty
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'host'))
from client import Client,USBLink
root=Path(__file__).parents[1]
master,slave=os.openpty();tty.setraw(slave);port=os.ttyname(slave)
sim=subprocess.Popen([str(root/'tests/controller_sim')],stdin=master,stdout=master,stderr=subprocess.PIPE)
async def check_python():
 c=Client(USBLink(port));await c.open();await c.arm();await c.velocity(400)
 await asyncio.sleep(1.4) # Longer than board watchdog; heartbeat must preserve ownership.
 assert (await c.status())['speed']==[400]*4
 await c.velocity(-200,3);assert (await c.status())['speed']==[400,400,-200,400]
 await c.close()
async def check_stopped():
 c=Client(USBLink(port));await c.open();s=await c.status();assert not s['armed'] and s['speed']==[0]*4,s;await c.close()
try:
 asyncio.run(check_python())
 result=subprocess.run([str(root/'host_macos/bin/pcb_mac_control'),'--serial',port,'--mode','simulate','--demo'],capture_output=True,text=True,timeout=10)
 assert result.returncode==0,result.stderr
 assert 'PCB 返回:' not in result.stderr,result.stderr
 assert 'V1600' in result.stdout or 'V1809' in result.stdout or '导管前进' in result.stdout,result.stdout
 asyncio.run(check_stopped())
 print('PASS: Python USB + native macOS host, firmware identity/arming, >1s heartbeat, independent axes, stop/disarm via PTY')
finally:
 sim.terminate();sim.wait(timeout=2);os.close(master);os.close(slave)
