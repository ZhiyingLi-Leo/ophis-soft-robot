import copy
import json
import os
import pty
import select
import struct
import tempfile
import threading
import time
import tty
import unittest
from pathlib import Path
from unittest.mock import patch
from server import Engine, Serial, frame, Handler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError

CONFIG = Path(__file__).resolve().parent.parent/'system_control'/'cylinders.conf.example'
class Tests(unittest.TestCase):
    def setUp(self): self.e = Engine(CONFIG, testing=True)
    def tearDown(self): self.e.close()
    def command(self, action, **kw): self.e.action(dict(action=action, client='test', **kw))
    def test_gamepad_partial_devices_neutral_and_timeout(self):
        self.command('connect_axis',axis=1)
        self.command('gamepad_start')
        self.assertEqual(self.e.gamepad['axes'],[1])
        self.command('gamepad',values=[1,1,1,1])
        self.assertFalse(self.e.axes[0]['moving'])
        self.assertTrue(self.e.axes[1]['moving'])
        self.assertFalse(self.e.axes[2]['moving'])
        self.command('gamepad',values=[0,0,0,0])
        self.assertFalse(self.e.axes[1]['moving'])
        self.assertTrue(self.e.axes[1]['armed'])
        self.command('gamepad',values=[0,0,-1,0])
        self.assertGreaterEqual(self.e.axes[1]['target'],0)
        self.e.gamepad['deadline']=0
        self.e.tick(.1)
        self.assertFalse(self.e.gamepad['active'])
        self.assertFalse(self.e.axes[1]['armed'])
        with self.assertRaises(ValueError): self.command('gamepad',values=[0,0,1,0])

    def test_gamepad_stop_and_late_connection(self):
        self.command('gamepad_start')
        self.command('connect_axis',axis=0)
        self.command('gamepad',values=[0,1,0,0])
        self.assertFalse(self.e.axes[0]['moving'])
        self.command('gamepad_stop')
        self.command('gamepad_start')
        self.command('stop_axis',axis=0)
        self.command('gamepad',values=[0,1,0,0])
        self.assertFalse(self.e.axes[0]['moving'])
        with self.assertRaises(ValueError): self.command('gamepad',values=[0,float('nan'),0,0])
        self.assertFalse(self.e.gamepad['active'])

    def test_gamepad_propulsion_independent_and_manual_stop(self):
        self.command('connect_prop')
        self.command('gamepad_start')
        self.command('gamepad',values=[-.5,1,1,1],speed=400)
        self.assertEqual(self.e.prop['targets'],[-200]*4)
        self.command('stop_prop')
        self.command('gamepad',values=[1,0,0,0])
        self.assertEqual(self.e.prop['targets'],[0]*4)
        self.command('stop_all')
        self.assertFalse(self.e.gamepad['active'])

    def test_hardware_default_and_no_simulation_api(self):
        with patch('server.Serial', side_effect=AssertionError('must not open')):
            e=Engine(CONFIG)
            self.assertEqual(e.state()['mode'], 'hardware')
            self.assertFalse(any(a['online'] for a in e.axes))
            with self.assertRaises(ValueError): e.action(dict(action='mode',mode='simulation',client='x'))
            e.close()
    def test_four_motor_status_while_driving(self):
        from unittest.mock import Mock
        self.e.mode='hardware'
        port=Mock()
        def reply(cmd, prefix):
            if cmd=='STATUS': return 'STATUS '+json.dumps(dict(speed=[100,200,-100,-200],current_ma=[500,600,700,800],fault=0,latched=0))
            return prefix
        port.line.side_effect=reply
        self.e.prop_serial=port
        self.e.prop.update(online=True,armed=True,backend='pcb',speed=100,deadline=time.monotonic()+10)
        self.e.tick(.1)
        p=self.e.state()['propulsion']
        self.assertEqual([m['speed'] for m in p['motors']],[100,200,-100,-200])
        self.assertEqual(p['motors'][3]['current_ma'],800)
        self.assertLess(p['telemetry_age'],1)
        port.line.assert_any_call('STATUS','STATUS ')

    def test_no_hardware_on_start(self):
        with patch('server.Serial', side_effect=AssertionError('must not open')), \
             patch('device_plugins.uvc._device_list', side_effect=AssertionError('must not discover')), \
             patch('device_plugins.uvc.subprocess.Popen', side_effect=AssertionError('must not open')):
            e=Engine(CONFIG, testing=True)
            e.scan()
            e.action(dict(action='connect_axis',axis=0,client='x'))
            self.assertFalse(e.state()['modules']['camera.side_view']['state']['online'])
            e.close()
    def test_independent_axis_and_move(self):
        self.command('connect_axis', axis=1)
        self.assertEqual([a['online'] for a in self.e.axes], [False, True, False])
        self.assertFalse(self.e.axes[1]['armed'])
        with self.assertRaises(ValueError): self.command('move',axis=1,value=1)
        self.command('arm',axis=1); self.command('move',axis=1,value=.1)
        for _ in range(10): self.e.tick(.1)
        self.assertAlmostEqual(self.e.axes[1]['position'], .096)
        self.assertFalse(self.e.axes[1]['moving'])
        self.assertFalse(self.e.prop['online'])
    def test_atomic_group_and_limits(self):
        self.command('connect_axis',axis=0); self.command('arm',axis=0)
        with self.assertRaises(ValueError): self.command('move_group', values=[1,1,1])
        self.assertEqual(self.e.axes[0]['target'],0)
        self.command('arm',axis=0)
        with self.assertRaises(ValueError): self.command('move',axis=0,value=-1)
        self.assertFalse(self.e.axes[0]['armed'])
        self.command('arm',axis=0)
        with self.assertRaises(ValueError): self.command('move',axis=0,value=float('nan'))
        with self.assertRaises(ValueError): self.command('connect_axis',axis=-1)
        self.command('arm',axis=0)
        for i in (1,2): self.command('connect_axis',axis=i); self.command('arm',axis=i)
        with self.assertRaises(ValueError): self.command('move_group',values=[1,17,2])
        self.assertTrue(all(a['target']==0 for a in self.e.axes))
    def test_lease_and_expiry(self):
        self.command('connect_axis',axis=0); self.command('arm',axis=0); self.command('move',axis=0,value=5)
        with self.assertRaises(ValueError): self.e.action(dict(action='arm',axis=0,client='other'))
        self.e.heartbeat=time.monotonic()-2; self.e.tick(.1)
        self.assertFalse(self.e.axes[0]['armed']); self.assertFalse(self.e.axes[0]['moving'])

    def test_slow_cycle_uses_elapsed_time_with_bounded_lead(self):
        self.command('connect_axis',axis=0); self.command('arm',axis=0)
        self.command('move',axis=0,value=5)
        self.e.tick(.2)
        self.assertAlmostEqual(self.e.axes[0]['command'],.1)
        before=self.e.axes[0]['position']
        self.e.tick(10)
        self.assertLessEqual(self.e.axes[0]['command']-before,.108)

    def test_lease_expiring_during_query_prevents_motion_write(self):
        from unittest.mock import Mock
        clock=[100.]
        bus=Mock()
        def query(*args):
            clock[0]+=2
            return dict(position=0,target=0,temperature=30)
        bus.cylinder.side_effect=query
        self.e.mode='hardware'; self.e.buses={'fake':bus}; self.e.axis_ports={0:'fake'}
        self.e.owner='test'; self.e.heartbeat=100.
        self.e.axes[0].update(online=True,armed=True,moving=True,target=5,move_started=100.)
        with patch('server.time.monotonic',side_effect=lambda:clock[0]): self.e.tick(.1)
        self.assertEqual(bus.cylinder.call_args_list,[unittest.mock.call(1)])
        self.assertFalse(self.e.axes[0]['armed'])
        self.assertIsNone(self.e.owner)

    def test_motor_deadline_rechecked_after_slow_cylinder_query(self):
        from unittest.mock import Mock
        clock=[100.]
        bus=Mock()
        def query(*args):
            clock[0]+=.4
            return dict(position=0,target=0,temperature=30)
        bus.cylinder.side_effect=query
        self.e.mode='hardware'; self.e.buses={'fake':bus}; self.e.axis_ports={0:'fake'}
        self.e.axes[0]['online']=True
        self.e.prop.update(online=True,armed=True,backend='sim',targets=[100,0,0,0],deadlines=[100.35,0,0,0])
        with patch('server.time.monotonic',side_effect=lambda:clock[0]): self.e.tick(.1)
        self.assertEqual(self.e.prop['targets'],[0]*4)
        self.assertFalse(self.e.prop['armed'])
    def test_mode_disconnect(self):
        self.command('connect_axis',axis=0); self.command('arm',axis=0)
        self.command('mode',mode='hardware')
        self.assertFalse(any(a['online'] or a['armed'] for a in self.e.axes))
    def test_arm_group_is_atomic_and_does_not_move(self):
        self.command('connect_axis',axis=0)
        with self.assertRaises(ValueError):self.command('arm_group')
        self.assertFalse(self.e.axes[0]['armed'])
        for i in [1,2]:self.command('connect_axis',axis=i)
        self.command('arm_group')
        self.assertTrue(all(a['armed'] for a in self.e.axes))
        self.assertFalse(any(a['moving'] or a['owned'] for a in self.e.axes))

    def test_heartbeat_not_blocked_by_serial_lock(self):
        self.e.owner='test';self.e.heartbeat=time.monotonic()-2
        with self.e.lock:
            t=threading.Thread(target=lambda:self.command('heartbeat'))
            t.start();t.join(.2)
            self.assertFalse(t.is_alive())
            self.assertLess(time.monotonic()-self.e.heartbeat_time(),.3)
        self.e.tick(.1)
        self.assertEqual(self.e.owner,'test')

    def test_other_page_cannot_replace_heartbeat_or_auto_stop(self):
        self.command('connect_axis',axis=0);self.command('arm',axis=0)
        self.command('heartbeat'); stamp=self.e.heartbeat_seen
        self.e.action(dict(action='heartbeat',client='background'))
        self.assertEqual(self.e.heartbeat_seen,stamp)
        self.e.action(dict(action='automatic_stop',client='background'))
        self.assertTrue(self.e.axes[0]['armed'])
        self.command('automatic_stop')
        self.assertTrue(self.e.axes[0]['armed'])
        self.command('arm',axis=0)
        self.e.action(dict(action='stop_all',client='background'))
        self.assertFalse(self.e.axes[0]['armed'])

    def test_keepalive_renews_only_current_owner_and_never_starts_motor(self):
        self.command('connect_prop');self.command('motor',motor=0,value=100)
        http=ThreadingHTTPServer(('127.0.0.1',0),Handler);http.engine=self.e
        threading.Thread(target=http.serve_forever,daemon=True).start()
        def post(client):
            req=Request(f'http://127.0.0.1:{http.server_port}/api/action',data=json.dumps(dict(action='heartbeat',client=client,wait=True,renew_motors=True)).encode(),headers={'Content-Type':'application/json'})
            with urlopen(req) as response:
                self.assertEqual(response.status,200)
                self.assertEqual(json.load(response)['accepted'],client=='test')
        try:
            previous=self.e.prop['deadlines'][0]
            post('other');self.assertEqual(self.e.prop['deadlines'][0],previous)
            post('test');self.assertGreater(self.e.prop['deadlines'][0],previous)
            self.command('stop_prop');post('test')
            self.assertEqual(self.e.prop['targets'],[0]*4)
        finally:http.shutdown();http.server_close()

    def test_propulse_deadman(self):
        self.command('connect_prop'); self.command('propulsion',value=400)
        self.e.prop['deadlines']=[0.]*4; self.e.tick(.1)
        self.assertEqual(self.e.prop['speed'],0)
    def test_independent_motor_and_group(self):
        self.command('connect_prop')
        self.command('motor',motor=0,value=200)
        self.command('motor',motor=2,value=-300)
        self.assertEqual(self.e.prop['targets'],[200,0,-300,0])
        self.command('motor_stop',motor=0)
        self.assertEqual(self.e.prop['targets'],[0,0,-300,0])
        self.command('propulsion',value=-400)
        self.assertEqual(self.e.prop['targets'],[-400]*4)
        self.e.prop['deadlines'][1]=0
        self.command('motor_renew');self.e.tick(.01)
        self.assertEqual(self.e.prop['targets'],[-400,0,-400,-400])
        self.command('stop_prop');self.command('motor_renew')
        self.assertEqual(self.e.prop['targets'],[0]*4)
        with self.assertRaises(ValueError):self.command('motor',motor=4,value=200)
        with self.assertRaises(ValueError):self.command('motor',motor=0,value=1601)
        self.e.prop['backend']='nano'
        with self.assertRaises(ValueError):self.command('motor',motor=0,value=200)

    def test_dual_group_only_commands_selected_axes(self):
        for i in range(3):
            self.command('connect_axis', axis=i)
        for i in (2, 0):
            self.command('arm', axis=i)
        self.command('move_group', axes=[2, 0], values=[2, 1])
        first, parked, third = self.e.axes
        self.assertEqual((first['target'], third['target']), (1, 2))
        self.assertEqual(first['move_started'], third['move_started'])
        self.assertFalse(parked['armed'] or parked['moving'])
        self.assertEqual(parked['target'], 0)
        with self.assertRaises(ValueError):
            self.command('move_group', axes=[2, 2], values=[3, 4])
        self.assertEqual((first['target'], third['target']), (1, 2))
        with self.assertRaises(ValueError):
            self.command('move_group', axes=[2, 0], values=[3, 17])
        self.assertFalse(any(a['armed'] or a['moving'] for a in self.e.axes))

    def test_pcb_independent_wire_commands(self):
        from unittest.mock import Mock
        port=Mock();port.line.side_effect=lambda command,prefix:prefix
        self.e.prop_serial=port;self.e.prop.update(online=True,backend='pcb')
        self.command('motor',motor=1,value=250)
        self.command('motor',motor=3,value=-150)
        self.command('motor_stop',motor=1)
        port.line.assert_any_call('A1','OK A1')
        port.line.assert_any_call('W0,250,0,-150','OK AXES')
        self.assertEqual(port.line.call_args.args,('W0,0,0,-150','OK AXES'))

    def test_script_run_stop_and_revocation(self):
        http=ThreadingHTTPServer(('127.0.0.1',0),Handler);http.engine=self.e
        self.e.experiment.port=http.server_port
        threading.Thread(target=http.serve_forever,daemon=True).start()
        try:
            self.command('connect_prop')
            self.command('script_run',command='python scripts/motor_run.py --motor 2 --speed -200 --seconds 10')
            token=self.e.experiment.token
            deadline=time.monotonic()+4
            while time.monotonic()<deadline and not self.e.prop['targets'][1]:
                self.command('heartbeat');time.sleep(.02)
            self.assertEqual(self.e.prop['targets'],[0,-200,0,0])
            with self.assertRaises(ValueError):self.command('motor',motor=0,value=100)
            self.command('script_stop')
            self.assertEqual(self.e.prop['targets'],[0]*4)
            self.assertIsNone(self.e.experiment.token)
            req=Request(f'http://127.0.0.1:{http.server_port}/api/script',data=json.dumps(dict(action='motor',motor=0,value=100)).encode(),headers={'Content-Type':'application/json','X-Experiment-Token':token})
            with self.assertRaises(HTTPError):urlopen(req)
            deadline=time.monotonic()+3
            while self.e.experiment.process and time.monotonic()<deadline:time.sleep(.02)
            self.assertIsNone(self.e.experiment.process)
            self.command('script_run',command='python scripts/motor_run.py --motor 1 --speed 100 --seconds .15')
            deadline=time.monotonic()+3
            while self.e.experiment.process and time.monotonic()<deadline:
                self.command('heartbeat');time.sleep(.02)
            self.assertEqual(self.e.experiment.status,'completed')
            self.assertEqual(self.e.prop['targets'],[0]*4)
            self.command('script_run',command='python scripts/status.py')
            deadline=time.monotonic()+3
            while self.e.experiment.process and time.monotonic()<deadline:
                self.command('heartbeat');time.sleep(.02)
            self.assertEqual(self.e.experiment.status,'completed')
            self.assertIn('propulsion',self.e.experiment.state()['output'])
            with self.assertRaises(ValueError):self.command('script_run',command='python ../server.py')
            with self.assertRaises(ValueError):self.command('script_run',command='sh scripts/status.py')
        finally:
            self.e.experiment.cancel();http.shutdown();http.server_close()

    def test_transient_query_recovers_without_motion(self):
        from unittest.mock import Mock
        bus=Mock()
        replies=[TimeoutError('echo only'),dict(position=20,target=20,temperature=30)]
        bus.cylinder.side_effect=replies
        self.e.mode='hardware';self.e.buses={'fake':bus};self.e.axis_ports={0:'fake'}
        a=self.e.axes[0];a.update(online=True,armed=True,moving=True,position=.16)
        self.e.tick(.1)
        self.assertTrue(a['online']);self.assertFalse(a['armed']);self.assertFalse(a['moving'])
        self.assertEqual(a['comm_failures'],1)
        self.e.tick(.1)
        self.assertTrue(a['online']);self.assertFalse(a['armed']);self.assertFalse(a['moving'])
        self.assertEqual(a['error'],'');self.assertEqual(a['comm_recoveries'],1)
        self.assertEqual(bus.cylinder.call_args_list,[unittest.mock.call(1),unittest.mock.call(1)])
        bus.cylinder.side_effect=TimeoutError('echo only')
        for _ in range(3):self.e.tick(.1)
        self.assertFalse(a['online'])
        self.assertIn('连续 3 次',a['error'])

    def test_one_axis_hardware_bus(self):
        class Bus:
            def __init__(self): self.commands=[]
            def cylinder(self, device, address=None, values=()):
                self.commands.append((device,address,values))
                if device==2: raise TimeoutError('echo only')
                return dict(position=0,target=values[-1] if address==0x25 else 0,temperature=30)
            def close(self): pass
        bus=Bus(); self.e.mode='hardware'; self.e.buses={'fake':bus}; self.e.axis_ports={0:'fake',1:'fake'}
        for a in self.e.axes[:2]: a.update(online=True,armed=True,owned=True)
        self.e.tick(.1)
        self.assertTrue(self.e.axes[0]['online']); self.assertFalse(self.e.axes[1]['online'])
        self.assertIn('停止未确认',self.e.axes[1]['error'])
        self.assertFalse(self.e.axes[0]['armed'])
        self.assertTrue(any(x[0]==1 and x[1]==0x1a for x in bus.commands))

    def test_unconfirmed_stop_can_be_retried(self):
        from unittest.mock import Mock
        bus=Mock(); bus.cylinder.side_effect=[TimeoutError('stop lost'),dict(position=0,target=0,temperature=30)]
        self.e.mode='hardware'; self.e.buses={'fake':bus}; self.e.axis_ports={0:'fake'}
        a=self.e.axes[0]; a.update(online=True,armed=True,owned=True,moving=True)
        self.e.stop_axis(0)
        self.assertFalse(a['armed']); self.assertTrue(a['owned'])
        self.assertIn('停止未确认',a['error'])
        self.e.stop_axis(0)
        self.assertFalse(a['owned'])
        self.assertEqual(bus.cylinder.call_count,2)

    def test_protocol_rejects_corruption_wrong_id_and_echo(self):
        def packet(device=2,fault=0):
            p=bytearray([0xaa,0x55,15,device,0x30])+struct.pack('<HhhHhHbB',0,125,124,0,0,0,30,fault)
            p.append(sum(p[2:])&255)
            return p
        port=Serial.__new__(Serial)
        def exchange(request,decode,**kwargs):
            self.assertEqual(request,frame(2,0x30))
            bad=packet();bad[-1]^=1
            data=bytearray(request)+packet(1)+bad
            self.assertIsNone(decode(data))
            valid=packet()
            self.assertIsNone(decode(data+valid[:19]))
            return decode(data+valid)
        port.exchange=exchange
        self.assertEqual(port.cylinder(2)['position'],124)
        port.exchange=lambda request,decode,**kw:decode(packet(fault=1))
        with self.assertRaisesRegex(RuntimeError,'故障码 1'):port.cylinder(2)

    def test_echo_only_times_out_without_resending(self):
        master,slave=pty.openpty();tty.setraw(slave);seen=[]
        def device():
            request=os.read(master,128);seen.append(request);os.write(master,request)
        t=threading.Thread(target=device);t.start()
        port=None
        try:
            port=Serial(os.ttyname(slave))
            with self.assertRaisesRegex(TimeoutError,'接收 6 字节'):port.cylinder(2)
            self.assertEqual(seen,[frame(2,0x30)])
            self.assertFalse(select.select([master],[],[],0)[0])
        finally:
            if port:port.close()
            t.join(1);os.close(master);os.close(slave)
    def test_discovery_partial_bus_and_ambiguity(self):
        calls=[]
        class Partial:
            def __init__(self,path,baud=115200): self.path=path
            def close(self): pass
            def cylinder(self,device,address=None,values=()):
                calls.append((self.path,device,address))
                if device != 2: raise TimeoutError('missing')
                return dict(position=-4,target=-4,temperature=30)
        self.e.mode='hardware'
        with patch.object(self.e,'ports',return_value=['/fake/a','/fake/b']), patch('server.Serial',Partial):
            self.e.scan()
            self.assertEqual([d['ids'] for d in self.e.devices],[[2],[2]])
            with self.assertRaises(ValueError): self.e.connect_axis(1,None)
            self.e.connect_axis(1,'/fake/b')
            self.assertTrue(self.e.axes[1]['online'])
            self.assertFalse(self.e.axes[0]['online'])
            self.assertFalse(self.e.axes[1]['armed'])
            self.assertTrue(all(addr is None for _,_,addr in calls))
            self.e.axes[1]['armed']=True
            with self.assertRaises(ValueError): self.e.scan()

    def test_axis_pair_partial_start_stops_d405_without_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            self.e.camera.data_root=Path(folder)
            self.command('heartbeat')
            self.e.camera.status.update(online=True,last_frame_time=time.monotonic())
            with self.assertRaisesRegex(ValueError,'仅由实验脚本'):
                self.command('axis_record_start',label='trial',group='test')
            with self.assertRaisesRegex(ValueError,'新鲜图像'):
                self.command('axis_record_start',label='trial',group='test',_script=True)
            self.assertFalse(self.e.camera.snapshot()['recording'])
            self.assertFalse(self.e.side_view.snapshot()['recording'])
            path, = Path(folder).glob('test/*/frames.jsonl')
            self.assertEqual(json.loads(path.read_text().splitlines()[-1])['kind'],'end')
            self.assertFalse(any(a['armed'] or a['moving'] for a in self.e.axes))

    def test_axis_recorders_stop_on_escape_and_lease_expiry(self):
        from test_uvc import catheter
        from pretest_data import read_side_capture

        self.command('heartbeat')
        with tempfile.TemporaryDirectory() as folder:
            self.e.camera.data_root=Path(folder)
            for action in ('stop_all','lease'):
                now=time.monotonic()
                self.e.camera.status.update(online=True,last_frame_time=now)
                self.e.side_view._publish(dict(host_receive_monotonic=now,frame_number=1 if action=='stop_all' else 2,
                                               fps=30),catheter())
                result=self.e.action(dict(action='axis_record_start',client='test',label=action,group='test',_script=True))
                recorder=self.e.side_view.recording
                if action=='lease':
                    self.e.heartbeat=time.monotonic()-2
                    self.e.expire_control()
                else:
                    self.command('stop_all')
                recorder['thread'].join(2)
                path=Path(result['session_path'])
                self.assertEqual(read_side_capture(path),[])
                self.assertFalse(self.e.camera.snapshot()['recording'])
                self.assertFalse(self.e.side_view.snapshot()['finalizing'])
                self.assertTrue(self.e.side_view.snapshot()['online'])

    def test_side_configuration_is_independent_and_auto_locate_rechecks_experiment(self):
        from test_uvc import catheter

        original = copy.deepcopy(self.e.camera.settings)
        now=time.monotonic()
        self.e.side_view._publish(dict(host_receive_monotonic=now,frame_number=1,fps=30),catheter())
        real_candidate=self.e.side_view.roi_candidate
        with patch.object(self.e.experiment,'process',None):
            def candidate():
                result=real_candidate()
                self.e.experiment.process=object()
                return result
            with patch.object(self.e.side_view,'roi_candidate',side_effect=candidate):
                with self.assertRaisesRegex(ValueError,'实验状态已变化'):
                    self.command('device',device='camera.side_view',operation='auto_locate')
            with self.assertRaisesRegex(ValueError,'实验运行中'):
                self.command('device',device='camera.side_view',operation='configure',arguments={'white_sat':12})
        self.assertEqual(self.e.side_view.snapshot()['settings']['roi'],[0.,0.,1.,1.])
        self.command('device',device='camera.side_view',operation='auto_locate')
        self.assertLess(self.e.side_view.snapshot()['settings']['roi'][2],1.)
        self.assertEqual(self.e.camera.settings,original)

    def test_http_d405_devices(self):
        with ThreadingHTTPServer(('127.0.0.1', 0), Handler) as server:
            server.engine = self.e
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            url = f'http://127.0.0.1:{server.server_port}/api/camera/d405/devices'
            devices = [{'serial': '123456', 'name': 'Intel RealSense D405', 'firmware': '5.16'}]
            try:
                with patch.object(self.e.camera, 'discover', return_value=devices) as discover, \
                     patch.object(self.e.camera, 'connect') as connect, \
                     patch.object(self.e.side_view, 'discover') as side_discover:
                    with urlopen(url) as response:
                        self.assertEqual(response.status, 200)
                        self.assertEqual(json.load(response), devices)
                    discover.assert_called_once_with()
                    connect.assert_not_called()
                    side_discover.assert_not_called()
                    with self.assertRaises(HTTPError) as denied:
                        urlopen(Request(url, headers={'Origin': 'https://evil.example'}))
                    self.assertEqual(denied.exception.code, 403)
                    self.assertEqual(discover.call_count, 1)
                with patch.object(self.e.camera, 'discover', side_effect=RuntimeError('枚举 D405 设备失败')):
                    with self.assertRaises(HTTPError) as unavailable:
                        urlopen(url)
                    with unavailable.exception as response:
                        self.assertEqual(response.code, 503)
                        self.assertEqual(json.load(response), {'error': '枚举 D405 设备失败'})
            finally:
                server.shutdown()
                thread.join()

    def test_http_local_only(self):
        import urllib.request
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler); server.engine=self.e
        t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
        url=f'http://127.0.0.1:{server.server_port}'
        try:
            self.assertEqual(json.load(urlopen(url+'/api/state'))['mode'],'simulation')
            for route in ('/scene.js','/side-camera.js','/vendor/three.module.js','/vendor/GLTFLoader.js'):
                with urlopen(url+route) as response:
                    self.assertIn('text/javascript', response.headers['Content-Type'])
                    self.assertGreater(len(response.read()), 100)
            for route in ('/models/cylinders.glb','/models/advancer.glb'):
                with urlopen(url+route) as response:
                    self.assertEqual(response.headers['Content-Type'], 'model/gltf-binary')
                    self.assertEqual(response.read(4), b'glTF')
            with self.assertRaises(HTTPError): urlopen(url+'/models/../../server.py')
            with patch.object(self.e.side_view,'discover',return_value=[{'id':'ace','name':'Insta360 Ace','type':'external'}]):
                self.assertEqual(json.load(urlopen(url+'/api/camera/side-view/devices'))[0]['id'],'ace')
            with self.assertRaises(HTTPError) as absent:
                urlopen(url+'/api/camera/side-view/preview')
            self.assertEqual(absent.exception.code,404)
            preview=(b'test JPEG',{'roi':[.1,.2,.9,.8]})
            with patch.object(self.e.side_view,'preview',return_value=preview):
                with urlopen(url+'/api/camera/side-view/preview') as response:
                    self.assertEqual(response.read(),preview[0])
                    self.assertEqual(json.loads(response.headers['X-Camera-Settings']),preview[1])
            req=Request(url+'/api/camera/side-view/devices',headers={'Host':'evil.example'})
            with self.assertRaises(HTTPError): urlopen(req)

            req=Request(url+'/api/action',data=b'{"action":"stop_all"}',headers={'Content-Type':'application/json','Origin':'https://evil.example'})
            with self.assertRaises(HTTPError): urlopen(req)
            req=Request(url+'/api/state',headers={'Host':'evil.example'})
            with self.assertRaises(HTTPError): urlopen(req)
        finally: server.shutdown();server.server_close();t.join()
    def test_serial_echo_fragmentation_and_read_only(self):
        master,slave=pty.openpty(); tty.setraw(slave); path=os.ttyname(slave); queries=[]
        def device():
            packet=os.read(master,128);queries.append(packet)
            reply=bytearray([0xaa,0x55,15,2,0x30])+struct.pack('<HhhHhHbB',0,-4,-4,0,0,0,30,0)
            reply.append(sum(reply[2:])&255)
            os.write(master,packet+reply[:7]);time.sleep(.01);os.write(master,reply[7:])
        t=threading.Thread(target=device);t.start()
        try:
            port=Serial(path); result=port.cylinder(2);port.close()
            self.assertEqual(result['position'],-4);self.assertEqual(queries,[frame(2,0x30)])
        finally:t.join(timeout=1);os.close(master);os.close(slave)
if __name__=='__main__': unittest.main()
