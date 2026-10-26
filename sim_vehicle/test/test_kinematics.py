"""四轮转向运动学测试。"""

import math
import unittest

from sim_vehicle.kinematics import (
    CORNERS,
    WHEELBASE_M,
    WHEEL_RADIUS_M,
    WHEEL_TRACK_M,
    four_wheel_state,
)


class KinematicsTest(unittest.TestCase):

    def assertClose(self, actual, expected):
        self.assertTrue(
            math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9),
            msg="%r != %r" % (actual, expected),
        )

    def test_geometry_matches_isaac_stage(self):
        self.assertClose(WHEELBASE_M, 1.521)
        self.assertClose(WHEEL_TRACK_M, 1.306)
        self.assertClose(WHEEL_RADIUS_M, 0.29)

    def test_straight_motion_keeps_all_wheels_straight(self):
        state = four_wheel_state(0.6, 0.0, 0)

        self.assertClose(state.yaw_rate_rad_s, 0.0)
        for corner in CORNERS:
            self.assertClose(state.steer_rad[corner], 0.0)
            self.assertClose(
                state.wheel_rad_s[corner], 0.6 / WHEEL_RADIUS_M)

    def test_single_ackermann_uses_front_steering_only(self):
        state = four_wheel_state(0.6, math.radians(15.0), 0)

        self.assertGreater(state.steer_rad["FL"], 0.0)
        self.assertGreater(state.steer_rad["FR"], 0.0)
        self.assertGreater(state.steer_rad["FL"], state.steer_rad["FR"])
        self.assertClose(state.steer_rad["RL"], 0.0)
        self.assertClose(state.steer_rad["RR"], 0.0)
        self.assertGreater(
            state.wheel_rad_s["FR"], state.wheel_rad_s["FL"])
        self.assertClose(state.effective_wheelbase_m, WHEELBASE_M)

    def test_symmetric_ackermann_steers_front_and_rear_opposite(self):
        state = four_wheel_state(0.6, math.radians(15.0), 1)

        self.assertGreater(state.steer_rad["FL"], 0.0)
        self.assertGreater(state.steer_rad["FR"], 0.0)
        self.assertLess(state.steer_rad["RL"], 0.0)
        self.assertLess(state.steer_rad["RR"], 0.0)
        self.assertClose(
            state.steer_rad["FL"], -state.steer_rad["RL"])
        self.assertClose(
            state.steer_rad["FR"], -state.steer_rad["RR"])
        self.assertClose(
            state.effective_wheelbase_m, WHEELBASE_M * 0.5)

    def test_right_turn_mirrors_left_turn(self):
        left = four_wheel_state(0.6, math.radians(15.0), 1)
        right = four_wheel_state(0.6, math.radians(-15.0), 1)

        mirror = {"FL": "FR", "FR": "FL", "RL": "RR", "RR": "RL"}
        for corner in CORNERS:
            opposite = mirror[corner]
            self.assertClose(
                right.steer_rad[corner], -left.steer_rad[opposite])
            self.assertClose(
                right.wheel_rad_s[corner],
                left.wheel_rad_s[opposite],
            )

    def test_modes_without_defined_twist_are_stopped(self):
        for steer_mode in (2, 3):
            with self.subTest(steer_mode=steer_mode):
                state = four_wheel_state(
                    0.6, math.radians(30.0), steer_mode)
                self.assertClose(state.yaw_rate_rad_s, 0.0)
                for corner in CORNERS:
                    self.assertClose(state.steer_rad[corner], 0.0)
                    self.assertClose(state.wheel_rad_s[corner], 0.0)

    def test_zero_speed_has_zero_wheel_speed(self):
        state = four_wheel_state(0.0, math.radians(30.0), 1)

        self.assertClose(state.yaw_rate_rad_s, 0.0)
        for corner in CORNERS:
            self.assertClose(state.wheel_rad_s[corner], 0.0)


if __name__ == "__main__":
    unittest.main()
