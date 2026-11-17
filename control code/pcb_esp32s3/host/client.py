"""USB CDC / BLE NUS client for the PCB1 ESP32-S3 firmware, not the Nano."""
from __future__ import annotations
import asyncio
import json
import time

SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

class USBLink:
    def __init__(self, port): self.port_name = port
    async def open(self):
        import serial
        self.port = serial.Serial(self.port_name, 115200, timeout=.1, write_timeout=.5)
        await asyncio.sleep(.5)
        self.port.reset_input_buffer()
    async def write(self, line):
        def send():
            self.port.write((line + "\n").encode("ascii")); self.port.flush()
        await asyncio.to_thread(send)
    async def readline(self):
        # Serial.readline may return a partial line at timeout. Keep fragments.
        if not hasattr(self, "buffer"): self.buffer = bytearray()
        while True:
            part = await asyncio.to_thread(self.port.read, 256)
            self.buffer.extend(part)
            if b"\n" in self.buffer:
                line, _, rest = self.buffer.partition(b"\n"); self.buffer = bytearray(rest)
                return line.decode("ascii").strip()
            if len(self.buffer) > 4096: raise RuntimeError("USB response too long")
    async def close(self): self.port.close()

class BLELink:
    def __init__(self, address=None):
        self.address=address; self.queue=asyncio.Queue(); self.buffer=bytearray()
    async def open(self):
        from bleak import BleakClient, BleakScanner
        self.loop=asyncio.get_running_loop()
        device=self.address
        if not device:
            devices=await BleakScanner.discover(timeout=5,return_adv=True)
            matches=[d for d,a in devices.values() if SERVICE in [x.lower() for x in (a.service_uuids or [])]]
            if len(matches)!=1:
                raise RuntimeError(f"发现{len(matches)}块推进器板；请用--scan后显式指定--ble地址，避免连接错板")
            device=matches[0]
        self.client=BleakClient(device,disconnected_callback=lambda _: self.loop.call_soon_threadsafe(self.queue.put_nowait,None))
        await self.client.connect()
        await self.client.start_notify(TX,self._notify)
    def _notify(self,_sender,data):
        self.buffer.extend(data)
        while b"\n" in self.buffer:
            line,_,rest=self.buffer.partition(b"\n");self.buffer=bytearray(rest)
            self.queue.put_nowait(bytes(line))
        if len(self.buffer)>4096:
            self.buffer.clear();self.queue.put_nowait(None)
    async def write(self,line):
        payload=(line+"\n").encode("ascii")
        for i in range(0,len(payload),20):await self.client.write_gatt_char(RX,payload[i:i+20],response=True)
    async def readline(self):
        line=await self.queue.get()
        if line is None:raise ConnectionError("BLE已断开")
        return line.decode("ascii").strip()
    async def close(self):
        if self.client.is_connected:await self.client.disconnect()

class DryLink:
    """Host-only command demonstration; not an electrical or firmware simulation."""
    def __init__(self):self.q=asyncio.Queue();self.speed=[0]*4;self.armed=0
    async def open(self):pass
    async def write(self,line):
        print("DRY TX",line)
        if line=="INFO":reply="INFO PCB1_S3 DRY_RUN"
        elif line=="STATUS":reply="STATUS "+json.dumps(dict(board="PCB1_S3",speed=self.speed,armed=self.armed,external=1,fault=0,latched=0,event="HOST_DEMO"))
        elif line=="?":reply=f"V{self.speed[0]}"
        else:
            reply="OK "+line
            if line in ("S","V0","A0","E"):self.speed=[0]*4
            if line=="A1":self.armed=1
            if line in ("A0","E"):self.armed=0
            if line.startswith("V"):self.speed=[int(line[1:])]*4
            if line.startswith("M"):
                a,v=map(int,line[1:].split(','));self.speed[a-1]=v
        self.q.put_nowait(reply)
    async def readline(self):return await self.q.get()
    async def close(self):pass

class Client:
    def __init__(self,link):
        self.link=link;self.lock=asyncio.Lock();self.owned=False;self.failure=None;self.task=None
    async def open(self):
        await self.link.open()
        try:
            info=await self.command("INFO")
            if not info.startswith("INFO PCB1_S3 "):raise RuntimeError("设备不是新PCB固件，拒绝解锁")
        except BaseException:
            await self.link.close();raise
        self.task=asyncio.create_task(self._heartbeat())
        return info
    async def command(self,line):
        if not line or len(line)>94 or any(ord(c)<32 or ord(c)>126 for c in line):raise ValueError("无效单行命令")
        async with self.lock:
            await self.link.write(line)
            deadline=time.monotonic()+1.5
            while True:
                reply=await asyncio.wait_for(self.link.readline(),max(.01,deadline-time.monotonic()))
                if reply.startswith("READY"):continue
                if reply.startswith("ERR"):raise RuntimeError(reply)
                break
            if line=="A1":self.owned=True
            if line in ("A0","E"):self.owned=False
            return reply
    async def arm(self):return await self.command("A1")
    async def velocity(self,speed,axis=None):
        if self.failure:raise RuntimeError(f"连接失效：{self.failure}")
        if not -16000<=speed<=16000:raise ValueError("速度超限")
        if axis is not None and axis not in (1,2,3,4):raise ValueError("轴号必须1..4")
        return await self.command(f"M{axis},{speed}" if axis else f"V{speed}")
    async def status(self):return json.loads((await self.command("STATUS")).removeprefix("STATUS "))
    async def _heartbeat(self):
        while True:
            await asyncio.sleep(.2)
            if self.owned:
                try:await self.command("H")
                except Exception as e:
                    self.failure=e;self.owned=False
                    # Board watchdog also stops independently if the link cannot deliver S.
                    try:await self.command("S")
                    except Exception:pass
    async def close(self):
        if self.task:
            self.task.cancel()
            try:await self.task
            except asyncio.CancelledError:pass
        try:
            if self.owned:
                await self.command("S");await self.command("A0")
        finally:await self.link.close()
