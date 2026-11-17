import unittest

from propulsion_pcb_control.control_logic import ControlConfig, axis_to_speed, limit_speed_change


class ControlLogicTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = ControlConfig(max_speed=3200, deadzone=0.1, max_acceleration=1000.0)

    def test_deadzone_and_limits(self) -> None:
        self.assertEqual(axis_to_speed(0.05, self.config), 0)
        self.assertEqual(axis_to_speed(-0.05, self.config), 0)
        self.assertEqual(axis_to_speed(1.0, self.config), 3200)
        self.assertEqual(axis_to_speed(-1.0, self.config), -3200)
        self.assertEqual(axis_to_speed(2.0, self.config), 3200)

    def test_speed_ramp_and_reverse(self) -> None:
        self.assertEqual(limit_speed_change(0, 3200, 0.1, self.config), 100)
        self.assertEqual(limit_speed_change(100, -3200, 0.1, self.config), 0)
        self.assertEqual(limit_speed_change(0, -3200, 0.1, self.config), -100)
        self.assertEqual(limit_speed_change(50, 100, 0.0, self.config), 50)

    def test_invalid_config(self) -> None:
        with self.assertRaises(ValueError):
            ControlConfig(max_speed=0).validate()


if __name__ == "__main__":
    unittest.main()
