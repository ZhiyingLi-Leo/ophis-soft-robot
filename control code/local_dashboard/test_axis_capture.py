import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from axis_capture import AxisCapture, main


class CaptureRobot:
    def __init__(self, directory):
        self.directory = directory
        self.now = 10.
        self.axes = [dict(id=i, position=0, target=0, minimum=0, maximum=16, speed=.5,
                          moving=False, armed=False, online=True, error='', age=0, stroke=16)
                     for i in (1, 2, 3)]
        self.armed = []
        self.moves = []
        self.path = None
        self.records = []
        self.recorded_frames = 0

    def measurement(self):
        return dict(shape_3d=dict(valid=True, angle_deg=sum(a['position'] for a in self.axes)))

    def state(self):
        camera = dict(online=True, error='', frame_age=.1, recording=bool(self.path), record_error='',
                      settings={}, measurement_settings={}, measurement=self.measurement(),
                      recorded_frames=self.recorded_frames)
        return dict(axes=[dict(a, last_seen=self.now) for a in self.axes], camera=camera)

    def arm(self, axis):
        self.axes[axis - 1]['armed'] = True
        self.armed.append(axis)

    def stop_cylinder(self, axis):
        self.axes[axis - 1]['armed'] = False

    def move(self, axis, mm):
        self.move_group((axis,), [mm])

    def move_all(self, values):
        self.move_group((1, 2, 3), values)

    def move_group(self, axes, values):
        self.moves.append((tuple(axes), tuple(values)))
        for axis, value in zip(axes, values):
            assert self.axes[axis - 1]['armed']
            self.axes[axis - 1].update(position=value, target=value)

    def device(self, name, operation, *, label, group):
        assert (name, operation) == ('camera.d405', 'record_start')
        self.path = self.directory / group / label
        self.path.mkdir(parents=True)
        self.records = []
        self.recorded_frames = 0
        self.next_frame = self.now + .2
        return dict(session_path=str(self.path))

    def annotate(self, **note):
        self.records.append(dict(kind='annotation', host_monotonic=self.now, note=note))

    def wait(self, seconds):
        self.now += seconds
        while self.path and self.next_frame <= self.now:
            self.recorded_frames += 1
            self.records.append(dict(kind='frame', file=f'frame_{self.recorded_frames:06d}.npz',
                                     frame_number=self.recorded_frames, host_receive_monotonic=self.next_frame,
                                     measurement=self.measurement(), telemetry=dict(axes=[
                                         dict(a, last_seen=self.now) for a in self.axes])))
            self.next_frame += .2

    def record_stop(self):
        (self.path / 'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in self.records))
        self.path = None


class DualCaptureRobot(CaptureRobot):
    def __init__(self, directory, fault=''):
        super().__init__(directory)
        self.fault = fault
        self.side_frames = []
        self.phase = ''
        self.settings = {'roi':[.1,.15,.9,.85]}

    def measurement(self):
        if self.phase in ('T0','target_hold'):
            return dict(shape_3d=dict(valid=False,angle_deg=None,reason='downward bend'))
        return super().measurement()

    def state(self):
        state = super().state()
        active = bool(self.path and self.phase in ('T0','target_hold'))
        if active and self.fault == 'd405-stall':
            state['camera']['frame_age'] = 1.1
        if active and self.fault == 'axis-feedback':
            state['axes'][2]['age'] = 1.1
        side = dict(online=not (active and self.fault == 'side-disconnect'), error='', frame_age=.1,
                    settings=self.settings, measurement_settings=self.settings,
                    recording=bool(self.path), finalizing=False, recorded_frames=len(self.side_frames),
                    valid_recorded_frames=sum(frame['measurement']['valid'] for frame in self.side_frames),
                    record_error='disk error' if active and self.fault=='writer-error' else '',
                    measurement=self.side_measurement())
        if active and self.fault == 'side-stall':
            side['frame_age'] = 1.1
        if active and self.fault == 'settings':
            side['settings'] = {'roi':[0.,0.,1.,1.]}
        state['modules'] = {'camera.side_view':{'state':side}}
        return state

    def side_measurement(self):
        active = self.phase in ('T0','target_hold')
        valid = self.fault != 'reference' and not (self.fault=='side-contour' and active)
        return dict(valid=valid,angle_deg=12*self.axes[2]['position'] if valid else None)

    def axis_record_start(self,label,group):
        result = self.device('camera.d405','record_start',label=label,group=group)
        self.side_frames = []
        self.phase = ''
        (self.path/'side_view').mkdir()
        return result

    def annotate(self, **note):
        self.phase = note['kind']
        super().annotate(**note)

    def wait(self,seconds):
        count = len(self.records)
        super().wait(seconds)
        if self.fault=='writer-stall' and self.phase in ('T0','target_hold'):
            return
        for frame in self.records[count:]:
            self.side_frames.append(dict(kind='frame',measurement=self.side_measurement(),
                                         host_receive_monotonic=frame['host_receive_monotonic']+.025))

    def axis_record_stop(self):
        index = self.path/'side_view'/'frames.jsonl'
        end = dict(kind='end',frames=len(self.side_frames))
        index.write_text(''.join(json.dumps(record)+'\n' for record in [*self.side_frames,end]))
        self.record_stop()


class AxisCaptureTests(unittest.TestCase):
    def test_first_segment_has_15_rounds_with_fixed_recording_windows(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            robot = CaptureRobot(Path(tmp))
            points = [(mm, 0, 0) for mm in range(1, 6)]
            batch = AxisCapture(robot, (1,), points, 5)
            with patch('axis_capture.time.monotonic', side_effect=lambda: robot.now):
                batch.run()
            folder = Path(tmp) / batch.group
            rounds = json.loads((folder / 'points.json').read_text())['completed_rounds']
            self.assertEqual(len(rounds), 15)
            self.assertEqual(robot.armed, [1])
            self.assertEqual(robot.moves, [((1,), (q,)) for _ in range(3) for mm in range(1, 6) for q in (mm, 0)])
            self.assertFalse(any(axis['armed'] for axis in robot.axes))
            for entry in rounds:
                info = json.loads((folder / entry['path'] / 'round.json').read_text())
                self.assertEqual(info['status'], 'complete')
                events = info['events_monotonic_s']
                self.assertGreaterEqual(events['T0'] - events['reference_hold'], 4)
                self.assertGreaterEqual(events['return'] - events['target_hold'], 4)
                self.assertEqual(set(info['preview']['triggers']), {'T0', 'T1', 'T2'})

    def test_dual_and_triple_capture_use_group_commands(self):
        for ids, point in [((3, 1), (4, 0, 2)), ((1, 2, 3), (2, 4, 6))]:
            with self.subTest(ids=ids), tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
                robot = CaptureRobot(Path(tmp))
                with patch('axis_capture.time.monotonic', side_effect=lambda: robot.now):
                    AxisCapture(robot, ids, [point], sum(point)).run()
                self.assertEqual(tuple(robot.armed), ids)
                self.assertEqual(robot.moves, [(ids, tuple(point[i - 1] for i in ids)), (ids, (0,) * len(ids))] * 3)

    def test_dual_id3_completes_through_d405_and_side_contour_loss(self):
        for fault in ('','side-contour'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
                robot = DualCaptureRobot(Path(tmp),fault)
                batch = AxisCapture(robot,(3,),[(0,0,2)],2,side_view=True)
                with patch('axis_capture.time.monotonic',side_effect=lambda:robot.now):
                    batch.run()
                self.assertEqual(robot.moves,[((3,),(2,)),((3,),(0,))]*3)
                for entry in batch.batch['completed_rounds']:
                    info = json.loads((Path(tmp)/batch.group/entry['path']/'round.json').read_text())
                    self.assertEqual(info['valid_hold_angles'],0)
                    self.assertGreater(info['side_view']['hold_frames'],0)
                    self.assertEqual(info['side_view']['valid_hold_angles']>0,not fault)
                self.assertTrue(all(not axis['armed'] for axis in robot.axes))

    def test_dual_faults_stop_and_keep_incomplete_round(self):
        faults = {'d405-stall':'图像失效','side-stall':'侧视相机图像失效','side-disconnect':'侧视相机图像失效',
                  'writer-error':'disk error','writer-stall':'记录帧停止写入',
                  'settings':'侧视 ROI','axis-feedback':'ID3 反馈'}
        for fault,message in faults.items():
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
                robot=DualCaptureRobot(Path(tmp),fault)
                batch=AxisCapture(robot,(3,),[(0,0,2)],2,side_view=True)
                with patch('axis_capture.time.monotonic',side_effect=lambda:robot.now):
                    with self.assertRaisesRegex(RuntimeError,message):
                        batch.run()
                self.assertFalse(batch.batch['completed_rounds'])
                self.assertEqual(robot.moves,[((3,),(2,))])
                self.assertTrue(all(not axis['armed'] for axis in robot.axes))
                path, = (Path(tmp)/batch.group).glob('*/round.json')
                self.assertEqual(json.loads(path.read_text())['status'],'incomplete')

    def test_capture_requires_side_reference_and_legacy_keeps_3d_rule(self):
        for side,fault,message in [(True,'reference','侧视参考角'),(False,'','三维角持续丢失')]:
            with self.subTest(side=side), tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
                robot=DualCaptureRobot(Path(tmp),fault)
                batch=AxisCapture(robot,(3,),[(0,0,2)],2,side_view=side)
                with patch('axis_capture.time.monotonic',side_effect=lambda:robot.now):
                    with self.assertRaisesRegex(RuntimeError,message):
                        batch.run()
                self.assertEqual(bool(robot.moves),not side)
                self.assertFalse(batch.batch['completed_rounds'])

    def test_preview_never_opens_control_session(self):
        args = ['collect_dual.py', '--ids', '1', '3', '--stop-mm', '4', '--side-view']
        with patch('sys.argv', args), patch('axis_capture.Robot') as robot, redirect_stdout(io.StringIO()) as out:
            main(2)
        self.assertIn('共 12 轮', out.getvalue())
        robot.assert_not_called()
