import importlib.util
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from pretest_data import export_videos, summarize
from scripts.pretest import Pretest
from server import Engine

CONFIG = Path(__file__).parent.parent / 'system_control' / 'cylinders.conf.example'


def example_capture():
    frames = [dict(frame_number=i, file=f'frame_{i:06}.npz', host_receive_monotonic=i/5,
                   measurement=dict(projected_signed_deg=angle, curve_px=[[4, 10], [20, 14], [30, 21]]),
                   telemetry=dict(axes=[dict(id=1, position=i/10, last_seen=i/5-.01)]))
              for i, angle in enumerate([.1, 0, 2, 10, 90, 10, 3, 2, 0, .1, 0])]
    return frames, dict(T0=.2, target_hold=.6, **{'return':1.2}, T2=1.9)


class PretestTests(unittest.TestCase):
    def test_preview_confirmation_required_and_pause_is_atomic(self):
        engine = Engine(CONFIG, testing=True)
        self.addCleanup(engine.close)

        def action(name, **fields):
            return engine.action(dict(action=name,client='test',**fields))

        for i in range(3):
            action('connect_axis', axis=i)
        with self.assertRaisesRegex(ValueError, '预览'):
            action('script_run', command='python scripts/pretest.py --max-advance-mm 4')
        settings = engine.camera.settings
        engine.camera.status.update(online=True, last_frame_time=time.monotonic(), measurement={'valid':True})
        with self.assertRaisesRegex(ValueError, '新鲜、有效'):
            action('pretest_start', settings=settings, max_advance_mm=4)
        engine.camera.status['measurement_settings'] = settings
        with patch.object(engine.experiment, 'start') as start:
            action('pretest_start', settings=settings, max_advance_mm=4)
            start.assert_called_once_with('python scripts/pretest.py --max-advance-mm 4', 'test', pretest=True)
        self.assertFalse(any(a['moving'] for a in engine.axes))
        engine.experiment.status = 'running'
        engine.experiment.pretest = {'phase':'return'}
        action('pretest_pause')
        self.assertFalse(engine.experiment.pretest_progress({'phase':'prepare'}, begin_round=True))
        with self.assertRaisesRegex(ValueError, '本轮结束'):
            action('pretest_resume')
        engine.experiment.pretest_progress({'phase':'paused'})
        action('pretest_resume')
        self.assertTrue(engine.experiment.pretest_progress({'phase':'prepare'}, begin_round=True))

    def test_triggers_follow_one_capture_and_suppress_noisy_peaks(self):
        frames, events = example_capture()
        triggers, _, _ = summarize(frames, events, 0, .5)
        self.assertEqual(triggers['T0']['frame'], 1)
        self.assertEqual(triggers['T1']['relative_angle_deg'], 10)
        self.assertIn(triggers['T1']['frame'], (3, 4, 5))
        self.assertEqual(triggers['T1']['feedback'][0]['id'], 1)
        self.assertEqual(triggers['T2']['frame'], 9)
        quiet, _, _ = summarize(frames, events, 0, 20)
        self.assertEqual(quiet['T1'], {'status':'indistinguishable', 'frame':None})

    def test_preroll_starts_with_first_saved_frame(self):
        from types import SimpleNamespace
        elapsed = 0

        def wait(seconds):
            nonlocal elapsed
            elapsed += seconds

        def observe(_):
            recorded = 0 if elapsed < .75 else 1 if elapsed < .95 else 2
            return [], {'recorded_frames':recorded}, 0

        pretest = Pretest(SimpleNamespace(wait=wait), 4)
        pretest.reference = []
        with patch('scripts.pretest.time.monotonic', side_effect=lambda: elapsed), \
                patch.object(pretest, 'observe', side_effect=observe):
            pretest.wait_preroll([])
        self.assertGreaterEqual(elapsed, 1.35)
        self.assertLess(elapsed, 1.5)

    @unittest.skipUnless(importlib.util.find_spec('cv2') and importlib.util.find_spec('numpy'), 'vision dependencies')
    def test_raw_video_tracks_preserve_pixel_values(self):
        import cv2
        import numpy as np
        random = np.random.default_rng(1)
        color = random.integers(0, 256, size=(48, 64, 3), dtype=np.uint8)
        depth = random.integers(0, 65536, size=(48, 64), dtype=np.uint16)
        frames, events = example_capture()
        triggers, angles, _ = summarize(frames, events, 0, .5)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            for frame in frames:
                np.savez_compressed(path / frame['file'], color_bgr=color, native_depth=depth)
            export_videos(path, frames, angles, triggers)
            for name, expected in [('color_raw.mkv', color), ('depth_raw.mkv', depth)]:
                decoder = cv2.VideoCapture(str(path / name), cv2.CAP_FFMPEG)
                if expected.ndim == 2:
                    decoder.set(cv2.CAP_PROP_CONVERT_RGB, 0)
                ok, decoded = decoder.read()
                decoder.release()
                self.assertTrue(ok)
                np.testing.assert_array_equal(decoded, expected)
            self.assertGreater((path / 'angle_overlay.mp4').stat().st_size, 0)
