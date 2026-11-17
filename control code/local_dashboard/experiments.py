"""Local Python experiment runner. Scripts are trusted local programs, not sandboxed."""
import collections
import os
import secrets
import shlex
import signal
import subprocess
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCRIPTS = ROOT / 'scripts'

class Experiments:
    def __init__(self, engine):
        self.engine = engine
        self.process = None
        self.token = None
        self.client = None
        self.output = collections.deque(maxlen=600)
        self.status = 'idle'
        self.command = ''
        self.port = None

    def state(self):
        return dict(status=self.status, command=self.command, output=''.join(self.output)[-48000:],
                    scripts=sorted('scripts/'+p.name for p in SCRIPTS.glob('*.py')))

    def start(self, command, client):
        if self.process is not None: raise ValueError('已有实验运行中')
        args = shlex.split(command)
        if len(args)<2 or args[0] not in ('python', 'python3'): raise ValueError('格式：python scripts/name.py [参数]')
        path = (ROOT / args[1]).resolve()
        if path.parent != SCRIPTS.resolve() or path.suffix != '.py' or not path.is_file():
            raise ValueError('请选择 scripts 目录中的 Python 脚本')
        if len(args)>64 or any(len(x)>1024 for x in args): raise ValueError('参数过长')
        self.engine.stop()
        self.output.clear(); self.command=command; self.client=client
        self.token=secrets.token_urlsafe(32)
        env=os.environ.copy()
        env.update(OPHIS_URL=f'http://127.0.0.1:{self.port}', OPHIS_TOKEN=self.token,
                   PYTHONPATH=str(ROOT), PYTHONUNBUFFERED='1')
        try:
            self.process=subprocess.Popen([sys.executable,'-u',str(path),*args[2:]],cwd=ROOT,env=env,
                stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
        except Exception:
            self.token=None; raise
        self.status='running'
        reader=threading.Thread(target=self.collect,args=(self.process,),daemon=True)
        reader.start()
        threading.Thread(target=self.finish,args=(self.process,reader),daemon=True).start()

    def collect(self, process):
        import codecs
        decoder=codecs.getincrementaldecoder('utf-8')('replace')
        try:
            while True:
                chunk=os.read(process.stdout.fileno(),1024)
                if not chunk: break
                self.output.append(decoder.decode(chunk))
            self.output.append(decoder.decode(b'',final=True))
        finally: process.stdout.close()

    def finish(self, process, reader):
        code=process.wait()
        with self.engine.lock:
            if self.process is not process: return
            cancelled=self.token is None
            self.token=None
            self.engine.stop()
            self.status='stopping'
            try: os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError: pass
        reader.join(timeout=1)
        with self.engine.lock:
            self.status='stopped' if cancelled else ('completed' if code==0 else 'failed')
            self.output.append(f'\n[退出码 {code}]\n')
            self.process=None

    def cancel(self):
        self.token=None
        process=self.process
        if process is None: return
        self.status='stopping'
        try: os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError: pass
        def finish():
            try: process.wait(timeout=.5)
            except subprocess.TimeoutExpired: pass
            try: os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError: pass
        threading.Thread(target=finish,daemon=True).start()
