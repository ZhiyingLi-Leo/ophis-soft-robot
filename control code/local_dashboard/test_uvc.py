import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from device_plugins import uvc, vision
from pretest_data import read_side_capture


def catheter(turn=0, size=(320, 180), *, offset=(0, 0)):
    width, height = size
    scale = min(width/320, height/180)
    def point(x, y):
        return tuple(round(n*scale) for n in (x+offset[0], y+offset[1]))
    image = np.empty((height, width, 3), np.uint8)
    image[:] = (55, 115, 20)
    cv2.line(image, point(57,90), point(96,90), (0, 120, 255), round(11*scale))
    points = np.array([point(97,90), point(132,90), point(159,90+turn/3), point(182,90+turn)])
    cv2.polylines(image, [points], False, (235,235,235), round(11*scale))
    return image


class UvcTests(unittest.TestCase):
    def setUp(self):
        self.device = uvc.SideViewDevice(lambda: {'valid': True, 'axes': []})
        self.addCleanup(self.device.close)

    def publish(self, color, stamp, number=1, fps=30):
        self.device._publish(dict(host_receive_monotonic=stamp,frame_number=number,fps=fps), color)

    def test_rgb_signed_angle_and_color_failure(self):
        for direction in (-1,0,1):
            result, overlay = vision.analyze_rgb(catheter(24*direction), self.device.settings)
            self.assertTrue(result['valid'])
            self.assertEqual(len(result['curve_px']),21)
            self.assertEqual(overlay.shape,(180,320,3))
            if direction:
                self.assertGreater(result['angle_deg']*direction,18)
            else:
                self.assertAlmostEqual(result['angle_deg'],0,delta=4)
        settings = dict(self.device.settings, orange_min=[100,80,55],orange_max=[120,255,255])
        result, _ = vision.analyze_rgb(catheter(24), settings)
        self.assertFalse(result['valid'])
        self.assertIsNone(result['angle_deg'])

    def test_autolocate_expands_from_full_frame_and_pads_for_excursion(self):
        self.device.command('configure', {'roi':[.82,.10,.98,.35]})
        self.publish(catheter(20),time.monotonic())
        self.assertFalse(self.device.snapshot()['measurement']['valid'])
        roi = self.device.command('auto_locate',{})['settings']['roi']
        self.assertLessEqual(roi[0],.12)
        self.assertGreaterEqual(roi[2],.65)
        self.assertLessEqual(roi[1],.25)
        self.assertGreaterEqual(roi[3],.8)
        self.assertTrue(all(0 <= bound <= 1 for bound in roi))
        fresh, _ = vision.analyze_rgb(catheter(-32), self.device.settings)
        self.assertTrue(fresh['valid'])
        candidate = self.device.roi_candidate()
        self.device.command('configure', {'white_sat':60})
        with self.assertRaisesRegex(ValueError,'已变化'):
            self.device.apply_auto_roi(candidate)

    def test_autolocate_invalid_frame_preserves_applied_roi(self):
        old = self.device.snapshot()['settings']
        self.publish(np.zeros((180,320,3),np.uint8),time.monotonic())
        with self.assertRaisesRegex(ValueError,'未找到导管'):
            self.device.auto_locate()
        self.assertEqual(self.device.snapshot()['settings'],old)
        for args in ({'roi':[.8,0,1.1,1]}, {'orange_max':[180,255,255]}, {'white_sat':True}):
            with self.assertRaises(ValueError):
                self.device.command('configure',args)
            self.assertEqual(self.device.snapshot()['settings'],old)

    def test_record_raw_png_negotiated_format_and_lock(self):
        for width,height,fps in [(1920,1080,30),(1280,720,29.97)]:
            with self.subTest(width=width), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)
                color = catheter(22,(width,height))
                device = uvc.SideViewDevice(lambda:{'valid':True, 'axes': [{'id':3,'position':1.1}]})
                self.addCleanup(device.close)
                start = time.monotonic()
                with patch('device_plugins.uvc.time.monotonic',return_value=start) as clock:
                    info = dict(host_receive_monotonic=start,frame_number=1,fps=fps)
                    device._publish(info,color)
                    self.assertEqual(device.snapshot()['format'],dict(width=width,height=height,fps=fps))
                    device.record_start(path)
                    rec = device.recording
                    for op in ('configure','auto_locate'):
                        with self.assertRaisesRegex(ValueError,'记录结束'):
                            device.command(op,{'white_min':98})
                    clock.return_value=start+.25
                    device._publish(dict(info,host_receive_monotonic=start+.25,frame_number=2),color)
                    device.stop_recording('完成')
                    rec['thread'].join(4)
                self.assertFalse(device.snapshot()['finalizing'])
                metadata = json.loads((path/'side_view'/'metadata.json').read_text())
                self.assertEqual(metadata['format'],dict(width=width,height=height,fps=fps))
                self.assertEqual(metadata['event_index'],'../frames.jsonl')
                frames = read_side_capture(path)
                self.assertEqual(len(frames),1)
                self.assertEqual(frames[0]['frame_number'],2)
                self.assertEqual(frames[0]['telemetry']['axes'][0]['position'],1.1)
                np.testing.assert_array_equal(cv2.imread(str(path/'side_view'/frames[0]['file'])),color)

    def test_record_disk_error_and_stale_preview(self):
        with tempfile.TemporaryDirectory() as tmp, patch('device_plugins.uvc.time.monotonic') as clock:
            clock.return_value=30
            self.publish(catheter(),30)
            self.device.record_start(Path(tmp))
            rec = self.device.recording
            with patch('cv2.imwrite',return_value=False):
                clock.return_value=30.3
                self.publish(catheter(),30.3,2)
                self.device.stop_recording('完成')
                rec['thread'].join(2)
            self.assertIn('PNG 写入失败',self.device.snapshot()['record_error'])
            clock.return_value=33.5
            self.assertIsNone(self.device.preview())

    def test_discovery_selects_exact_id_after_reordering_and_rejects_missing(self):
        entries = [
            {'id':'face','name':'FaceTime HD','type':'built_in'},
            {'id':'phone','name':'iPhone','type':'continuity'},
            {'id':'ace','name':'Insta360 Ace','type':'external'}
        ]
        process=Mock()
        with patch('device_plugins.uvc._device_list', side_effect=[entries,list(reversed(entries)),entries[:2],entries]), \
             patch('device_plugins.uvc._helper',return_value='/test/helper'), \
             patch('device_plugins.uvc.subprocess.Popen',return_value=process) as opener:
            self.assertEqual(self.device.discover(),entries)
            with patch('device_plugins.uvc._packets',side_effect=RuntimeError('权限未授予')):
                self.device.connect('ace')
                self.device.thread.join(2)
            self.assertEqual(opener.call_args.args[0],['/test/helper','--capture','ace'])
            self.assertIn('权限未授予',self.device.snapshot()['error'])
            self.device.connect('ace')
            self.device.thread.join(2)
            self.assertEqual(opener.call_count,1)
            self.assertIn('已离线',self.device.snapshot()['error'])
            live = threading.Event()
            def packets(_, halt):
                yield dict(host_receive_monotonic=time.monotonic(),frame_number=1,fps=30), catheter()
                live.set()
                halt.wait(3)
            with patch('device_plugins.uvc._packets',side_effect=packets):
                self.device.connect('ace')
                self.assertTrue(live.wait(2))
                self.assertIsNotNone(self.device.preview())
                self.device.disconnect()
                self.device.thread.join(2)
            self.assertFalse(self.device.snapshot()['online'])
            self.assertIsNone(self.device.preview())

    def test_frame_protocol_preserves_bgr_and_rejects_incomplete_transport(self):
        halted = threading.Event()
        with tempfile.TemporaryFile() as stream:
            info = dict(width=2,height=1,bytes=8,frame_number=1,fps=30,host_receive_monotonic=7)
            stream.write((json.dumps(info)+'\n').encode() + bytes(range(8)))
            stream.seek(0)
            frames = uvc._packets(stream,halted)
            header, image = next(frames)
            self.assertEqual(header,info)
            self.assertEqual(image.tolist(),[[[0,1,2],[4,5,6]]])
            with self.assertRaisesRegex(RuntimeError,'进程已退出'):
                next(frames)
