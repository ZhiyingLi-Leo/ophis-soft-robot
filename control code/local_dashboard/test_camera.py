import importlib.util
import json
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

from device_plugins import macos_camera
from device_plugins.realsense import D405Device
from launch_d405 import launch
from sampling import read_plan,require_camera,wait_stable
from server import Engine

CONFIG=Path(__file__).parent.parent/'system_control'/'cylinders.conf.example'
VISION=bool(importlib.util.find_spec('cv2') and importlib.util.find_spec('numpy'))


class D405DiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.camera = D405Device(Path('unused'), Mock())
        self.sdk = Mock()
        self.sdk.camera_info = SimpleNamespace(name='name', serial_number='serial', firmware_version='firmware')
        self.query = self.sdk.context.return_value.query_devices

    def test_lists_only_d405_devices_without_starting_capture(self):
        def device(name, serial, firmware):
            hardware = Mock()
            hardware.get_info.side_effect = dict(name=name, serial=serial, firmware=firmware).__getitem__
            return hardware

        other = device('Intel RealSense D455', '455-1', '5.0')
        self.query.return_value = [other, device('Intel RealSense D405', '405-1', '5.16'),
                                   device('RealSense d405', '405-2', '5.17')]
        with patch.dict('sys.modules', {'pyrealsense2': self.sdk}), \
             patch.object(self.camera, '_capture') as capture:
            self.assertEqual(self.camera.discover(), [
                {'serial': '405-1', 'name': 'Intel RealSense D405', 'firmware': '5.16'},
                {'serial': '405-2', 'name': 'RealSense d405', 'firmware': '5.17'},
            ])
        self.query.assert_called_once_with()
        other.get_info.assert_called_once_with('name')
        self.sdk.pipeline.assert_not_called()
        capture.assert_not_called()
        self.assertFalse(self.camera.status['online'])

    def test_empty_list(self):
        self.query.return_value = []
        with patch.dict('sys.modules', {'pyrealsense2': self.sdk}):
            self.assertEqual(self.camera.discover(), [])

    def test_missing_sdk_error(self):
        with patch.dict('sys.modules', {'pyrealsense2': None}):
            with self.assertRaisesRegex(RuntimeError, '无法加载 pyrealsense2'):
                self.camera.discover()

    def test_query_failure_error(self):
        self.query.side_effect = OSError('USB unavailable')
        with patch.dict('sys.modules', {'pyrealsense2': self.sdk}):
            with self.assertRaisesRegex(RuntimeError, '枚举 D405 设备失败.*USB unavailable'):
                self.camera.discover()


class LauncherTests(unittest.TestCase):
    @patch('launch_d405.subprocess.Popen')
    @patch('launch_d405.open_workbench')
    @patch('launch_d405.bridge_listening', return_value=True)
    @patch('launch_d405.workbench_state', return_value=None)
    def test_reuses_bridge_without_requesting_sudo(self, state, listening, open_workbench, popen):
        self.assertEqual(launch(8765, None), 0)
        open_workbench.assert_called_once_with(8765, 'http://127.0.0.1:8765')
        popen.assert_not_called()

    @patch('launch_d405.subprocess.run')
    @patch('launch_d405.subprocess.Popen')
    @patch('launch_d405.open_workbench')
    @patch('launch_d405.bridge_listening', side_effect=[False, False])
    @patch('launch_d405.workbench_state', return_value=None)
    def test_does_not_open_browser_after_bridge_start_failure(self, state, listening, open_workbench, popen, compile_helper):
        popen.return_value.poll.return_value = 1
        self.assertEqual(launch(8765, None), 1)
        compile_helper.assert_called_once()
        open_workbench.assert_not_called()


@unittest.skipUnless(VISION, 'optional camera processing dependencies not installed')
class MacOSBridgeTests(unittest.TestCase):
    def setUp(self):
        root = tempfile.TemporaryDirectory()
        self.addCleanup(root.cleanup)
        self.helper = Path(root.name) / 'ophis-d405-capture'
        self.helper.touch()
        self.camera = D405Device(root.name, lambda: {})
        self.rs = Mock()
        self.rs.__spec__ = importlib.util.spec_from_loader('pyrealsense2', loader=None)
        for mock in (patch('sys.platform', 'darwin'), patch.dict('sys.modules', {'pyrealsense2': self.rs}),
                     patch.object(macos_camera, 'helper_path', return_value=self.helper)):
            mock.start()
            self.addCleanup(mock.stop)
        self.addCleanup(self.camera.close)

    def test_connect_auto_launches_and_disconnect_reaps_owned_bridge(self):
        process = Mock()
        process.poll.return_value = None
        conn = MagicMock()
        receiving = threading.Event()

        def receive(camera, serial, rs, **kwargs):
            self.assertIs(kwargs['conn'], conn)
            receiving.set()
            camera.halt.wait(3)

        with patch.object(macos_camera, 'connect_bridge', side_effect=[FileNotFoundError, conn]), \
             patch.object(macos_camera, 'receive', side_effect=receive) as receiver, \
             patch('device_plugins.realsense.subprocess.Popen', return_value=process) as popen:
            self.camera.connect('123456')
            self.assertTrue(receiving.wait(2), self.camera.snapshot())
            self.assertIs(self.camera.bridge_process, process)
            popen.assert_called_once()
            self.assertEqual(popen.call_args.args[0], ['/usr/bin/sudo', '-n', '--', str(self.helper), '123456'])
            self.assertEqual(popen.call_args.kwargs['stdin'], subprocess.DEVNULL)
            receiver.assert_called_once_with(self.camera, '123456', self.rs, conn=conn)
            self.camera.disconnect()
            self.camera.thread.join(3)
            self.assertFalse(self.camera.thread.is_alive())
            process.terminate.assert_called_once_with()
            process.wait.assert_called_once_with(timeout=2)
            self.assertIsNone(self.camera.bridge_process)
            self.assertEqual(self.camera.snapshot()['status'], '未接入')

    def test_close_reuses_manual_bridge_without_stopping_its_process(self):
        conn = MagicMock()
        receiving = threading.Event()

        def receive(camera, *_args, **_kwargs):
            receiving.set()
            camera.halt.wait(3)

        with patch.object(macos_camera, 'connect_bridge', return_value=conn), \
             patch.object(macos_camera, 'receive', side_effect=receive), \
             patch('device_plugins.realsense.subprocess.Popen') as popen:
            self.camera.connect()
            self.assertTrue(receiving.wait(2), self.camera.snapshot())
            self.camera.close()
            self.assertFalse(self.camera.thread.is_alive())
            conn.shutdown.assert_called_once()
            popen.assert_not_called()

    def test_sudo_password_failure_reports_chinese_guidance_without_sdk_fallback(self):
        process = Mock()
        process.poll.return_value = 1

        def launch(_args, **kwargs):
            kwargs['stderr'].write(b'sudo: a password is required\n')
            kwargs['stderr'].flush()
            return process

        with patch.object(macos_camera, 'connect_bridge', side_effect=ConnectionRefusedError), \
             patch('device_plugins.realsense.subprocess.Popen', side_effect=launch):
            self.camera.connect()
            self.camera.thread.join(3)
            self.assertFalse(self.camera.thread.is_alive())
            error = self.camera.snapshot()['error']
            self.assertIn('免密 sudo', error)
            self.assertIn('/etc/sudoers.d/ophis-d405', error)
            self.assertIn('start_d405.command', error)
            self.rs.context.assert_not_called()
            process.terminate.assert_not_called()
            self.assertIsNone(self.camera.bridge_process)


class PluginTests(unittest.TestCase):
    def test_registry_and_missing_camera_do_not_block_cylinders(self):
        engine=Engine(CONFIG,testing=True)
        try:
            self.assertEqual(set(engine.state()['modules']),{'cylinder.1','cylinder.2','cylinder.3','propulsion','camera.d405','camera.side_view'})
            def command(key,op,**arguments):return engine.action(dict(action='device',client='test',device=key,operation=op,arguments=arguments))
            command('cylinder.2','connect');command('cylinder.2','arm')
            with patch('device_plugins.realsense.dependencies',return_value={'pyrealsense2':False}):
                with self.assertRaisesRegex(ValueError,'依赖未安装'):command('camera.d405','connect')
            self.assertTrue(engine.axes[1]['armed'])
            command('cylinder.2','move',mm=.1)
            engine.tick(.2)
            self.assertGreater(engine.axes[1]['position'],0)
            self.assertFalse(engine.axes[0]['online'])
            with self.assertRaises(ValueError):command('camera.unknown','connect')
        finally:engine.close()

    def test_script_cannot_reconfigure_or_connect_devices(self):
        engine=Engine(CONFIG,testing=True)
        try:
            for name in ('camera.d405','camera.side_view','cylinder.1'):
                with self.assertRaisesRegex(ValueError,'连接与配置'):
                    engine.action(dict(action='device',device=name,operation='connect',client='test',_script=True))
            self.assertFalse(any(a['online'] for a in engine.axes))
        finally:engine.close()

    def test_configuration_is_atomic_and_record_requires_camera(self):
        with tempfile.TemporaryDirectory() as root:
            camera=D405Device(root,lambda:{})
            old=camera.snapshot()['settings']
            with self.assertRaises(ValueError):camera.command('configure',{'roi':[.8,0,.1,1]})
            with self.assertRaises(ValueError):camera.command('configure',{'roi':[0,0,1,1],'white_min':999})
            self.assertEqual(camera.snapshot()['settings'],old)
            with self.assertRaises(ValueError):camera.record_start('test')
            self.assertEqual(list(Path(root).iterdir()),[])

    def test_stability_checks_shape_not_only_angle(self):
        clock=[100.]
        class Robot:
            def state(self):
                shift=2 if int(round((clock[0]-100)*10))%2 else 0
                curve=[[float(i),float(shift),0.] for i in range(21)]
                return dict(camera=dict(online=True,error='',frame_age=0,recording=True,record_error='',frame_number=clock[0],
                    measurement=dict(valid=True,depth_fraction=1,surface_theta_deg=0,surface_curve_camera_mm=curve)),
                    axes=[dict(id=i,online=True,error='',armed=True,age=0,moving=False,position=0) for i in (1,2,3)])
            def wait(self,seconds):clock[0]+=seconds
        with patch('sampling.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaises(TimeoutError):wait_stable(Robot(),[0,0,0],1,.3,1,.05,.5)

    def test_sampling_defaults_to_validation_and_cleans_up_failed_motion(self):
        import contextlib
        import importlib.util
        import io
        from unittest.mock import Mock
        source=Path(__file__).parent/'scripts'/'sample_positions.py'
        spec=importlib.util.spec_from_file_location('sample_positions_test',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'plan.csv';path.write_text('q1_mm,q2_mm,q3_mm,hold_s\n0.1,0.1,0.1,0.01\n')
            args=['sample_positions.py','--plan',str(path),'--limits','0','1','0','1','0','1','--max-total-mm','1']
            with patch('sys.argv',args),patch.object(module,'Robot',side_effect=AssertionError('validation must not connect')),contextlib.redirect_stdout(io.StringIO()):
                module.main()
            calls=[]
            class Robot:
                def __enter__(self):return self
                def __exit__(self,*args):pass
                def state(self):
                    return dict(tolerance=.05,camera=dict(online=True,error='',frame_age=0,recording=True,record_error='',
                        measurement=dict(valid=True,depth_fraction=1,surface_theta_deg=0)),
                        axes=[dict(id=i,minimum=0,maximum=16,position=-.008,online=True,error='',age=0) for i in (1,2,3)])
                def record_start(self,label):return {'session_path':root}
                def record_stop(self):calls.append(('record','stop'))
                def annotate(self,**note):pass
                def device(self,name,op,**args):
                    calls.append((name,op))
                    if op=='move':raise RuntimeError('motion rejected')
            with patch('sys.argv',args+['--execute']),patch.object(module,'Robot',Robot),contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError,'motion rejected'):module.main()
            self.assertEqual(calls[-4:],[('cylinder.1','stop'),('cylinder.2','stop'),('cylinder.3','stop'),('record','stop')])
            self.assertTrue((Path(root)/'experiment_plan.json').is_file())

    def test_plan_bounds_and_finite_values(self):
        with tempfile.TemporaryDirectory() as root:
            p=Path(root)/'plan.csv'
            p.write_text('q1_mm,q2_mm,q3_mm,hold_s\n0.2,0.3,0.4,2\n')
            rows,_=read_plan(p,[0,1]*3,1)
            self.assertEqual(rows[0]['q'],[.2,.3,.4])
            with self.assertRaises(ValueError):read_plan(p,[0,1]*3,.5)
            p.write_text('q1_mm,q2_mm,q3_mm,hold_s\nnan,0,0,2\n')
            with self.assertRaises(ValueError):read_plan(p,[0,1]*3,1)

    def test_camera_fault_stops_sampling_observer(self):
        with self.assertRaisesRegex(RuntimeError,'失效'):
            require_camera({'camera':dict(online=True,error='',frame_age=2)})
        class Robot:
            def state(self):return {'camera':dict(online=False,error='USB disconnected',frame_age=0)}
        with self.assertRaisesRegex(RuntimeError,'USB disconnected'):
            wait_stable(Robot(),[0,0,0],1,.3,1,.05)


@unittest.skipUnless(VISION,'optional camera processing dependencies not installed')
class VisionTests(unittest.TestCase):
    def image(self):
        import cv2
        import numpy as np
        image=np.zeros((240,320,3),np.uint8);image[:]=(20,150,20)
        cv2.rectangle(image,(20,105),(70,135),(0,130,240),-1)
        cv2.rectangle(image,(71,110),(250,130),(220,220,220),-1)
        return image

    def test_straight_curve_and_invalid_depth_are_separate(self):
        import numpy as np
        from device_plugins.vision import analyze
        settings=dict(roi=[0,0,1,1],white_min=85,white_sat=100)
        original=self.image()
        result,overlay=analyze(original,np.zeros((240,320)),settings)
        self.assertTrue(result['valid'])
        self.assertEqual(len(result['curve_px']),21)
        self.assertLess(result['projected_theta_deg'],10)
        self.assertEqual(result['depth_fraction'],0)
        self.assertIsNone(result['surface_theta_deg'])
        self.assertLess(result['curve_px'][0][0],result['curve_px'][-1][0])
        x,y=np.rint(result['curve_px'][10]).astype(int)
        np.testing.assert_array_equal(overlay[y,x],original[y,x])

    def test_background_and_missing_orange_are_invalid(self):
        import numpy as np
        from device_plugins.vision import analyze
        image=self.image();image[:]=(20,150,20)
        settings=dict(roi=[0,0,1,1],white_min=85,white_sat=100)
        result,_=analyze(image,None,settings)
        self.assertFalse(result['valid'])
        image=self.image();image[:,0:71]=(20,150,20)
        self.assertFalse(analyze(image,None,settings)[0]['valid'])

    def test_depth_and_background_separation(self):
        import numpy as np
        from device_plugins.vision import analyze
        depth=np.full((240,320),200.);depth[110:131,71:251]=150.
        result,_=analyze(self.image(),depth,dict(roi=[0,0,1,1],white_min=85,white_sat=100))
        self.assertGreater(result['depth_fraction'],.9)
        self.assertAlmostEqual(result['background_separation_mm'],50)
        self.assertEqual(result['coordinate_frame'],'camera')

    def test_bend_out_of_image_plane_and_interior_gaps(self):
        import numpy as np
        from device_plugins.shape3d import fit_shape

        angles = np.linspace(0,np.deg2rad(70),21)
        radius = 30/angles[-1]
        center = np.column_stack((radius*np.sin(angles), np.zeros(21), 150+radius*(1-np.cos(angles))))
        tangent = np.column_stack((np.cos(angles), np.zeros(21), np.sin(angles)))
        radial = center - np.sum(center*tangent,axis=1)[:,None]*tangent
        surface = list(center - 3*radial/np.linalg.norm(radial,axis=1)[:,None])
        shaft = [[x,0,147] for x in np.linspace(-15,0,9)]
        surface[8:10] = [None,None]
        shape = fit_shape(surface,shaft,6,30)
        self.assertTrue(shape['valid'],shape['reason'])
        self.assertAlmostEqual(shape['angle_deg'],59.5,delta=2)
        np.testing.assert_allclose(shape['centerline_camera_mm'],center,atol=.5)
        self.assertNotIn(8,shape['observed_indices'])
        surface[-1] = None
        invalid = fit_shape(surface,shaft,6,30)
        self.assertFalse(invalid['valid'])
        self.assertIsNone(invalid['angle_deg'])

    def test_direction_window_keeps_raw_angle_and_clears_on_tracking_loss(self):
        import math
        from device_plugins.shape3d import BendDirectionHistory

        tracker = BendDirectionHistory()
        def shape(degrees):
            radians = math.radians(degrees)
            return dict(valid=True,angle_deg=degrees,shaft_axis={'direction':[1,0,0]},
                        tip_tangent=[math.cos(radians),0,math.sin(radians)])
        tracker.update(shape(30),1,{'roi':'A'})
        tracker.update(shape(30),1.17,{'roi':'A'})
        filtered = tracker.update(shape(65),1.34,{'roi':'A'})
        self.assertAlmostEqual(filtered['angle_deg'],30)
        self.assertEqual(filtered['raw_angle_deg'],65)
        self.assertEqual(filtered['direction_frames'],3)
        invalid = dict(valid=False,angle_deg=None)
        self.assertEqual(tracker.update(invalid,1.51,{'roi':'A'}),invalid)
        self.assertAlmostEqual(tracker.update(shape(65),1.68,{'roi':'A'})['angle_deg'],65)
        self.assertEqual(tracker.update(shape(15),1.85,{'roi':'B'})['direction_frames'],1)

    def test_reconstruction_rejects_background_depth_inside_the_tip_mask(self):
        import numpy as np
        from device_plugins.vision import analyze

        settings = dict(roi=[0,0,1,1],white_min=85,white_sat=100,length_mm=30,diameter_mm=6)
        depth = np.full((240,320),200.)
        depth[105:136,20:71] = 150.
        depth[110:131,71:251] = 150.
        deproject = lambda pixel,z:[(pixel[0]-160)*z/1000, (pixel[1]-120)*z/1000, z]
        result,_ = analyze(self.image(),depth,settings,deproject=deproject)
        self.assertTrue(result['shape_3d']['valid'],result['shape_3d']['reason'])
        self.assertAlmostEqual(result['shape_3d']['angle_deg'],0,delta=1)
        depth[110:131,232:251] = 200.
        result,_ = analyze(self.image(),depth,settings,deproject=deproject)
        self.assertEqual(result['depth_fraction'],1)
        self.assertFalse(result['shape_3d']['valid'])

    def test_capture_driver_with_sdk_double_saves_timestamps_and_stops(self):
        import numpy as np
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as root:
            camera=D405Device(root,lambda:{'valid':True})
            camera.status.update(online=True,last_frame_time=time.monotonic())
            path=Path(camera.record_start('sdk test')['session_path'])
            def telemetry():camera.halt.set();return {'valid':True,'axes':[]}
            camera.telemetry=telemetry
            intr=NS(width=320,height=240,fx=300.,fy=300.,ppx=160.,ppy=120.,model='none',coeffs=[0.]*5)
            video=Mock();video.as_video_stream_profile.return_value=video
            video.get_intrinsics.return_value=intr;video.intrinsics=intr;video.fps.return_value=30
            video.get_extrinsics_to.return_value=NS(rotation=[1,0,0,0,1,0,0,0,1],translation=[0,0,0])
            device=Mock();device.get_info.side_effect=lambda k:{'name':'Intel RealSense D405','serial':'test-only','firmware':'test'}[k]
            device.first_depth_sensor.return_value.get_depth_scale.return_value=.001
            depth=Mock();depth.get_data.return_value=np.full((240,320),150,np.uint16)
            color=Mock();color.get_data.return_value=self.image()
            for frame in (depth,color):
                frame.profile=video;frame.get_frame_number.return_value=7
                frame.get_timestamp.return_value=1234.5;frame.get_frame_timestamp_domain.return_value='hardware_clock'
            frames=Mock();frames.get_depth_frame.return_value=depth;frames.get_color_frame.return_value=color
            pipeline=Mock();profile=pipeline.start.return_value
            profile.get_device.return_value=device;profile.get_stream.return_value=video
            pipeline.wait_for_frames.return_value=frames
            rs=Mock();rs.pipeline.return_value=pipeline;rs.context.return_value.query_devices.return_value=[device]
            rs.camera_info=NS(name='name',serial_number='serial',firmware_version='firmware')
            rs.stream=NS(depth='depth',color='color');rs.format=NS(z16='z16',bgr8='bgr8')
            rs.align.return_value.process.return_value=frames
            rs.rs2_deproject_pixel_to_point.side_effect=lambda i,p,z:[(p[0]-160)*z/300,(p[1]-120)*z/300,z]
            with patch('sys.platform', 'linux'), patch.dict('sys.modules',{'pyrealsense2':rs}):
                camera._capture('')
            items=[json.loads(x) for x in (path/'frames.jsonl').read_text().splitlines()]
            self.assertEqual(items[0]['kind'],'frame')
            self.assertEqual(items[0]['frame_number'],7)
            self.assertEqual(items[0]['depth_timestamp_ms'],1234.5)
            self.assertEqual(len(items[0]['measurement']['surface_curve_camera_mm']),21)
            self.assertEqual(items[-1]['kind'],'end')
            pipeline.stop.assert_called_once()
            self.assertFalse(camera.snapshot()['online'])

    def test_record_preserves_raw_arrays_and_finalizes(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as root:
            telemetry=dict(valid=True,host_monotonic=10,axes=[dict(id=1,last_seen=9.9)])
            camera=D405Device(root,lambda:telemetry)
            camera.status.update(online=True,last_frame_time=time.monotonic(),calibration={'depth_scale_m':.0001})
            path=Path(camera.record_start('reference')['session_path'])
            values=np.arange(12,dtype=np.uint16).reshape(3,4)
            camera._save(dict(native_depth=values,aligned_depth=values,color_bgr=np.zeros((3,4,3),np.uint8)),
                         dict(host_receive_monotonic=time.monotonic(),frame_number=1,measurement={'valid':False}))
            camera.command('annotate',dict(note={'sample':0}))
            camera.stop_recording('test end')
            items=[json.loads(x) for x in (path/'frames.jsonl').read_text().splitlines()]
            self.assertEqual([x['kind'] for x in items],['frame','annotation','end'])
            self.assertEqual(items[0]['telemetry'],telemetry)
            with np.load(path/items[0]['file']) as arrays:np.testing.assert_array_equal(arrays['native_depth'],values)
            self.assertFalse(camera.snapshot()['recording'])
            with self.assertRaises(ValueError):camera.command('annotate',dict(note={}))

    def test_disk_error_is_visible_and_recording_stops(self):
        with tempfile.TemporaryDirectory() as root:
            camera=D405Device(root,lambda:{})
            camera.status.update(online=True,last_frame_time=time.monotonic())
            camera.record_start('disk failure')
            with patch('numpy.savez_compressed',side_effect=OSError('disk full')):
                camera._save({},dict(host_receive_monotonic=time.monotonic()))
            self.assertFalse(camera.snapshot()['recording'])
            self.assertEqual(camera.snapshot()['record_error'],'disk full')

if __name__=='__main__':unittest.main()
