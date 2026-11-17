"""New PCB USB CDC transport: identity check, arm, acknowledged commands, heartbeat."""
import time
import serial

class PCBSerial:
    def __init__(self,path,baud_rate=115200):
        self._port=serial.Serial(path,baud_rate,timeout=.1,write_timeout=.5)
        self.last_heartbeat=time.monotonic()
        try:
            time.sleep(.5);self._port.reset_input_buffer()
            if not self._request('INFO').startswith('INFO PCB1_S3 '):
                raise RuntimeError('设备不是PCB1_S3固件')
            self._request('A1');self._request('S')
        except BaseException:
            self._port.close();raise
    def _request(self,line):
        self._port.write((line+'\n').encode('ascii'));self._port.flush()
        deadline=time.monotonic()+.7;buf=bytearray()
        while time.monotonic()<deadline:
            buf.extend(self._port.read(1))
            if buf.endswith(b'\n'):
                reply=buf.decode('ascii').strip()
                if reply.startswith('ERR'):raise RuntimeError(reply)
                return reply
        raise TimeoutError('PCB应答超时；板端看门狗将停车')
    def send_speed(self,speed):
        if not -16000<=speed<=16000:raise ValueError('速度超限')
        self._request(f'V{int(speed)}')
    def keep_alive(self):
        if time.monotonic()-self.last_heartbeat>=.2:
            self._request('H');self.last_heartbeat=time.monotonic()
    def stop(self):self._request('S')
    def read_available(self):return ''
    def close(self):
        if self._port.is_open:
            try:self._request('S');self._request('A0')
            finally:self._port.close()
    def __enter__(self):return self
    def __exit__(self,*_):self.close()
