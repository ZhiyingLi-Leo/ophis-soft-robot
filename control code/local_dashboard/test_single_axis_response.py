import csv
import json
import tempfile
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from single_axis_response import summarize_response, summarize_side_points, summarize_side_round
from scripts import analyze_single


class ResponseTest(unittest.TestCase):
    def test_zero_each_repeat_and_match_hysteresis_by_actual_position(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'frames').mkdir()
            for target in (1, 2):
                for repeat in (1, 2, 3):
                    initial = 5*repeat
                    records = []
                    for phase, pos, delta in [('reference',0,0), ('advance',0,0), ('advance',target,4*target),
                                               ('hold',target,4*target), ('return',target,4*target+2), ('return',0,2)]:
                        curve = np.column_stack((np.linspace(0,30,21),np.full(21,pos),np.zeros(21)))
                        records.append(dict(phase=phase,angle_deg=initial+delta,valid=True,telemetry_valid=True,
                                            actual_mm=[pos,0,0],centerline_camera_mm=curve.tolist()))
                    path = root/'frames'/f'ID1_{target:02d}mm_R{repeat:02d}.jsonl'
                    path.write_text('\n'.join(json.dumps(row) for row in records))
            summarize_response(root)
            with (root/'resolution.csv').open() as f:
                results = list(csv.DictReader(f))
            self.assertEqual(results[1]['adjacent_repeat_ranges_disjoint'], 'True')
            self.assertEqual(results[2]['valid_repeats'], '0')
            self.assertEqual(results[2]['adjacent_repeat_ranges_disjoint'], '')
            with (root/'hysteresis.csv').open() as f:
                for row in csv.DictReader(f):
                    self.assertAlmostEqual(float(row['return_minus_advance_deg']), 2)
            shapes = json.loads((root/'steady_shapes.json').read_text())
            self.assertEqual(len(shapes), 6)
            np.testing.assert_allclose(shapes[0]['root_relative_change_mm'],0)

    def test_side_summary_wraps_signed_angles_and_preserves_coverage(self):
        timeline = [
            ('reference',-1,179,False),('reference',-.5,-179,False),('advance',0,-166,False),
            ('hold',1,-170,False),('hold',1.2,-168,False),('return',3.1,177,True),('return',3.3,None,True)
        ]
        rows = [dict(phase=phase, t_from_T0_s=t, angle_deg=angle, valid=angle is not None,
                     file=str(i), reference_position=reference)
                for i,(phase,t,angle,reference) in enumerate(timeline)]
        result = summarize_side_round(rows,cycle_end=3.4)
        self.assertAlmostEqual(result['hold_delta_deg'],11.)
        self.assertAlmostEqual(result['peak_delta_deg'],14.)
        self.assertAlmostEqual(result['return_residual_deg'],-3.)
        self.assertEqual(result['return_frames'],2)
        self.assertEqual(result['return_valid'],1)
        three = [dict(axis=3,target_mm=2,hold_delta_deg=v) for v in (-8.,-10.,-12.,None)]
        point, = summarize_side_points(three)
        self.assertEqual((point['recorded_repeats'],point['valid_repeats']),(4,3))
        self.assertEqual(point['repeat_sd_deg'],2.)

    def test_cli_analyzes_partial_id3_batch_from_raw_without_realsense(self):
        import cv2
        from device_plugins.uvc import SideViewDevice
        from test_uvc import catheter

        def write_jsonl(path, items):
            path.write_text(''.join(json.dumps(item)+'\n' for item in items))

        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)/'raw'
            path = source/'point'
            side = path/'side_view'
            side.mkdir(parents=True)
            (source/'points.json').write_text(json.dumps(dict(completed_rounds=[{'path':'point'}])))
            events = dict(reference_hold=100, T0=102, target_hold=105, **{'return':109}, T2=112)
            write_jsonl(path/'frames.jsonl',[dict(kind='frame',host_receive_monotonic=100.3)]+[
                dict(kind='annotation',note=dict(kind=kind),host_monotonic=stamp) for kind,stamp in events.items()])
            specs = [(99.9,0),(100,0),(101.99,0),(102,-29),(105,19),(108.8,None),(109,12),(111.8,0),(112,0)]
            info = dict(status='complete',ids=[3],target_mm=[0,0,2],repeat=1,reference_mm=[0,0,0],
                        frame_count=1,side_view=dict(frame_count=len(specs)))
            (path/'round.json').write_text(json.dumps(info))
            settings = SideViewDevice(lambda:None).settings
            (side/'metadata.json').write_text(json.dumps(dict(settings=settings,format=dict(width=320,height=180,fps=30))))
            frames = []
            for i,(stamp,angle) in enumerate(specs):
                file=f'frame_{i:06d}.png'
                image=catheter(angle) if angle is not None else np.zeros((180,320,3),np.uint8)
                self.assertTrue(cv2.imwrite(str(side/file),image))
                axes=[dict(id=n,position=2 if n==3 and 105<=stamp<109 else 0,online=True,
                           last_seen=stamp-(2 if angle is None else .02)) for n in (1,2,3)]
                frames.append(dict(kind='frame',file=file,frame_number=i,host_receive_monotonic=stamp,
                                   telemetry=dict(axes=axes,valid=True),measurement={'valid':False}))
            write_jsonl(side/'frames.jsonl',frames+[dict(kind='end',frames=len(frames))])
            dst=Path(tmp)/'results'
            cli=['analyze_single.py','--side-view',str(source),str(dst),'--workers','1']
            with patch.object(sys,'argv',cli),patch.dict(sys.modules,{'pyrealsense2':None}):
                analyze_single.main()
            result=dst/'side_view'
            rows = [json.loads(line) for line in (result/'frames'/'ID3_02mm_R01.jsonl').read_text().splitlines()]
            self.assertEqual([row['phase'] for row in rows],
                             ['before','reference','reference','advance','hold','hold','return','return','after'])
            self.assertEqual(rows[5]['actual_mm'],[None,None,None])
            self.assertFalse(rows[5]['valid'])
            with (result/'rounds.csv').open() as stream:
                row, = csv.DictReader(stream)
            self.assertEqual(row['peak_file'],'frame_000003.png')
            self.assertLess(float(row['peak_delta_deg']),0)
            self.assertGreater(float(row['hold_delta_deg']),0)
            self.assertEqual(float(row['actual_hold_mm']),2.)
            self.assertAlmostEqual(float(row['return_residual_deg']),0.)
            self.assertEqual(row['hold_valid'],'1')
            with (result/'points.csv').open() as stream:
                point, = csv.DictReader(stream)
            self.assertEqual(point['valid_repeats'],'1')
            self.assertEqual(point['repeat_sd_deg'],'')
