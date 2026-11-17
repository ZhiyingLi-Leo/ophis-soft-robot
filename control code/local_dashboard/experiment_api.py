"""Device API for trusted local experiment scripts. Connections are made in the UI."""
import json
import os
import threading
import time
from urllib.request import Request, build_opener, ProxyHandler

class Robot:
    def __init__(self):
        self.url=os.environ['OPHIS_URL']; self.token=os.environ['OPHIS_TOKEN']
        self.http=build_opener(ProxyHandler({}))
        self.done=threading.Event(); self.error=None
        self.thread=threading.Thread(target=self._renew,daemon=True); self.thread.start()

    def command(self, action, **values):
        if self.error: raise RuntimeError(self.error)
        req=Request(self.url+'/api/script',data=json.dumps(dict(action=action,**values)).encode(),
                    headers={'Content-Type':'application/json','X-Experiment-Token':self.token})
        try:
            with self.http.open(req,timeout=2) as response: return json.load(response)
        except Exception as e:
            message=e.read().decode() if hasattr(e,'read') else str(e)
            raise RuntimeError(message) from e

    def state(self):
        with self.http.open(self.url+'/api/state',timeout=2) as response: return json.load(response)

    def _renew(self):
        while not self.done.wait(.12):
            try: self.command('motor_renew')
            except Exception as e: self.error=str(e); return

    def motor(self, motor, speed): self.command('motor',motor=motor-1,value=speed)
    def motors(self, speed): self.command('propulsion',value=speed)
    def stop_motor(self, motor): self.command('motor_stop',motor=motor-1)
    def stop_motors(self): self.command('stop_prop')
    def arm(self, cylinder): self.command('arm',axis=cylinder-1)
    def move(self, cylinder, mm): self.command('move',axis=cylinder-1,value=mm)
    def move_all(self, values): self.command('move_group',values=values)
    def stop_cylinder(self, cylinder): self.command('stop_axis',axis=cylinder-1)
    def wait(self, seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if self.error: raise RuntimeError(self.error)
            time.sleep(min(.05,max(0,deadline-time.monotonic())))
    def wait_cylinder(self, cylinder, timeout=60):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            a=self.state()['axes'][cylinder-1]
            if not a['online'] or a['error']: raise RuntimeError(a['error'] or '电缸离线')
            if not a['moving']: return
            self.wait(.1)
        self.stop_cylinder(cylinder); raise TimeoutError('电缸等待超时')

    def device(self, name, operation, **arguments):
        return self.command('device',device=name,operation=operation,arguments=arguments).get('result')
    def camera_state(self): return self.state()['camera']
    def record_start(self, label=''): return self.device('camera.d405','record_start',label=label)
    def record_stop(self): return self.device('camera.d405','record_stop')
    def annotate(self, **note): return self.device('camera.d405','annotate',note=note)
    def close(self):
        self.done.set()
        self.thread.join(timeout=3)
    def __enter__(self): return self
    def __exit__(self, *_): self.close()
