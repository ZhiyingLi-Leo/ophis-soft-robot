import importlib.util
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from device_plugins.realsense import D405Device
from launch_d405 import launch
from sampling import read_plan,require_camera,wait_stable
from server import Engine

CONFIG=Path(__file__).parent.parent/'system_control'/'cylinders.conf.example'
VISION=bool(importlib.util.find_spec('cv2') and importlib.util.find_spec('numpy'))


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


class PluginTests(unittest.TestCase):
    def test_registry_and_missing_camera_do_not_block_cylinders(self):
        engine=Engine(CONFIG,testing=True)
        try:
            self.assertEqual(set(engine.state()['modules']),{'cylinder.1','cylinder.2','cylinder.3','propulsion','camera.d405'})
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
            for name in ('camera.d405','cylinder.1'):
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
            with patch('device_plugins.macos_camera.endpoint') as endpoint, patch.dict('sys.modules',{'pyrealsense2':rs}):
                endpoint.return_value.is_socket.return_value=False
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
