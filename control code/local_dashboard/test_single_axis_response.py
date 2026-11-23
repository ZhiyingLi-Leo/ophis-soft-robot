import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from single_axis_response import summarize_response


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
