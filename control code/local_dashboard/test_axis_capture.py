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

    def test_preview_never_opens_control_session(self):
        args = ['collect_dual.py', '--ids', '1', '3', '--stop-mm', '4']
        with patch('sys.argv', args), patch('axis_capture.Robot') as robot, redirect_stdout(io.StringIO()) as out:
            main(2)
        self.assertIn('共 12 轮', out.getvalue())
        robot.assert_not_called()
