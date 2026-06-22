import importlib.util
import sys
import types
import unittest
from datetime import datetime, timedelta
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
MY_CAR = ROOT / "python" / "my_car.py"


class FakeDrivingController:
    def set_enable_api_control(self, _enabled):
        self.enable_api_control = _enabled

    def __init__(self):
        self.half_road_limit = 10.5


def load_my_car_module():
    drive_controller = types.ModuleType("DrivingInterface.drive_controller")
    drive_controller.DrivingController = FakeDrivingController
    package = types.ModuleType("DrivingInterface")
    package.drive_controller = drive_controller
    sys.modules["DrivingInterface"] = package
    sys.modules["DrivingInterface.drive_controller"] = drive_controller

    spec = importlib.util.spec_from_file_location("my_car_under_test", MY_CAR)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def controls():
    return SimpleNamespace(steering=0.0, throttle=0.0, brake=0.0)


def obstacle(dist, to_middle):
    return {"dist": dist, "to_middle": to_middle}


def sensing(speed, to_middle, moving_angle, obstacles=None, collided=False, progress=40.0, angles=None):
    return SimpleNamespace(
        speed=speed,
        to_middle=to_middle,
        moving_angle=moving_angle,
        moving_forward=True,
        lap_progress=progress,
        track_forward_angles=angles or [0.0, 16.0, 22.0, 28.0, 30.0, 25.0, 18.0, 10.0],
        distance_to_way_points=[10.0] * 8,
        track_forward_obstacles=obstacles or [],
        opponent_cars_info=[],
        collided=collided,
    )


class V15RecoveryTests(unittest.TestCase):
    def setUp(self):
        module = load_my_car_module()
        self.car = module.DrivingClient()
        self.car.start_time = datetime.now() - timedelta(seconds=30)

    def tearDown(self):
        close = getattr(self.car, "__del__", None)
        if close:
            close()

    def test_close_obstacle_keeps_steering_when_crawling(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=0.04,
                to_middle=1.99,
                moving_angle=39.4,
                obstacles=[obstacle(3.03, 4.87), obstacle(8.5, -2.0)],
            ),
        )

        self.assertNotAlmostEqual(command.steering, 0.0, delta=0.02)

    def test_collision_enters_reverse_recovery_immediately(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=1.0,
                to_middle=2.3,
                moving_angle=-64.1,
                obstacles=[obstacle(3.93, 4.87)],
                collided=True,
            ),
        )

        self.assertLess(command.throttle, 0.0)
        self.assertEqual(self.car.accident_step, 1)

    def test_mild_overspeed_uses_lighter_brake(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=151.0,
                to_middle=0.4,
                moving_angle=0.0,
                obstacles=[],
                collided=False,
                progress=50.0,
                angles=[0.0, 5.0, 10.0, 10.0, 10.0, 8.0, 5.0, 0.0],
            ),
        )

        self.assertEqual(command.throttle, 0.0)
        self.assertLessEqual(command.brake, 0.35)
        self.assertGreaterEqual(command.brake, 0.25)

    def test_early_section_keeps_stronger_overspeed_brake(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=151.0,
                to_middle=0.4,
                moving_angle=0.0,
                obstacles=[],
                collided=False,
                progress=19.0,
                angles=[0.0, 5.0, 10.0, 10.0, 10.0, 8.0, 5.0, 0.0],
            ),
        )

        self.assertEqual(command.throttle, 0.0)
        self.assertGreaterEqual(command.brake, 0.5)

    def test_map31_early_left_obstacle_commits_right(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=153.8,
                to_middle=-0.41,
                moving_angle=-4.9,
                obstacles=[obstacle(27.23, -3.86), obstacle(169.95, 1.38)],
                collided=False,
                progress=18.28,
                angles=[0.0, 16.0, 22.0, 28.0, 30.0, 25.0, 18.0, 10.0],
            ),
        )

        self.assertGreater(self.car.target_offset, 0.5)
        self.assertLessEqual(self.car.target_offset, 1.3)
        self.assertGreater(command.steering, 0.18)
        self.assertLessEqual(command.steering, 0.25)
        self.assertEqual(command.throttle, 1.0)
        self.assertEqual(command.brake, 0.0)

    def test_map31_s_cut_does_not_chase_far_right_gap(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=148.62,
                to_middle=3.48,
                moving_angle=-1.6,
                obstacles=[obstacle(41.82, -3.86), obstacle(180.43, 1.38)],
                collided=False,
                progress=18.01,
                angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -3.0, -3.0, -3.0, -3.0, -7.0, -7.0, -7.0, -7.0, -7.0, -7.0, -6.0, -7.0],
            ),
        )

        self.assertGreaterEqual(self.car.target_offset, 2.8)
        self.assertLessEqual(self.car.target_offset, 3.3)
        self.assertLessEqual(abs(command.steering), 0.22)
        self.assertEqual(command.throttle, 1.0)
        self.assertEqual(command.brake, 0.0)

    def test_map31_s_cut_holds_existing_right_line(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=144.81,
                to_middle=4.24,
                moving_angle=-2.9,
                obstacles=[obstacle(51.43, -3.86), obstacle(190.04, 1.38)],
                collided=False,
                progress=17.74,
                angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -3.0, -3.0, -3.0, -3.0, -7.0, -7.0, -7.0, -7.0, -7.0, -7.0, -6.0],
            ),
        )

        self.assertGreaterEqual(self.car.target_offset, 2.8)
        self.assertLessEqual(self.car.target_offset, 3.3)
        self.assertGreaterEqual(command.steering, -0.05)
        self.assertLessEqual(abs(command.steering), 0.22)
        self.assertEqual(command.throttle, 1.0)
        self.assertEqual(command.brake, 0.0)

    def test_map31_launch_keeps_gentle_steering(self):
        self.car.start_time = datetime.now() - timedelta(seconds=1)

        self.car.control_driving(
            controls(),
            sensing(
                speed=5.79,
                to_middle=0.0,
                moving_angle=-1.6,
                obstacles=[obstacle(94.49, -0.98)],
                collided=False,
                progress=0.27,
                angles=[-2.0, -8.0, -14.0, -21.0, -24.0, -24.0, -24.0, -24.0],
            ),
        )

        command = self.car.control_driving(
            controls(),
            sensing(
                speed=7.67,
                to_middle=-0.01,
                moving_angle=-3.2,
                obstacles=[obstacle(94.29, -0.98)],
                collided=False,
                progress=0.27,
                angles=[-2.0, -8.0, -14.0, -21.0, -24.0, -24.0, -24.0, -24.0],
            ),
        )

        self.assertGreaterEqual(command.steering, -0.15)
        self.assertEqual(command.throttle, 1.0)
        self.assertEqual(command.brake, 0.0)

    def test_near_single_obstacle_requires_clear_same_side_lane_v10_2(self):
        self.car.start_time = datetime.now() - timedelta(seconds=5)

        command = self.car.control_driving(
            controls(),
            sensing(
                speed=75.07,
                to_middle=1.32,
                moving_angle=2.6,
                obstacles=[obstacle(20.57, 0.72)],
                collided=False,
                progress=4.84,
                angles=[0.0, -4.0, -8.0, -12.0, -16.0, -18.0, -18.0, -16.0],
            ),
        )

        self.assertGreaterEqual(self.car.target_offset, 3.6)
        self.assertGreater(command.steering, 0.0)

    def test_v10_2_close_collision_risk_cuts_throttle(self):
        self.car.start_time = datetime.now() - timedelta(seconds=18)

        command = self.car.control_driving(
            controls(),
            sensing(
                speed=73.72,
                to_middle=3.17,
                moving_angle=2.0,
                obstacles=[obstacle(13.41, 3.76), obstacle(177.0, 2.3)],
                collided=False,
                progress=10.22,
                angles=[0.0, -27.0, -36.0, -42.0, -45.0, -43.0, -38.0, -30.0],
            ),
        )

        self.assertEqual(command.throttle, 0.0)
        self.assertGreaterEqual(command.brake, 0.45)

    def test_near_single_obstacle_requires_clear_same_side_lane_v10_6(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=71.33,
                to_middle=-1.86,
                moving_angle=-8.0,
                obstacles=[obstacle(22.37, 0.47)],
                collided=False,
                progress=26.88,
                angles=[0.0, 8.0, 12.0, 16.0, 18.0, 18.0, 16.0, 12.0],
            ),
        )

        self.assertLessEqual(self.car.target_offset, -2.5)
        self.assertLess(abs(command.steering), 1.0)

    def test_v10_6_high_speed_close_collision_risk_brakes(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=156.68,
                to_middle=-1.46,
                moving_angle=-6.4,
                obstacles=[obstacle(16.94, -3.86), obstacle(160.0, 1.38)],
                collided=False,
                progress=68.55,
                angles=[0.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -7.0],
            ),
        )

        self.assertEqual(command.throttle, 0.0)
        self.assertGreaterEqual(command.brake, 0.35)

    def test_map31_high_speed_left_obstacle_pulls_off_right_edge_early(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=156.25,
                to_middle=4.30,
                moving_angle=6.3,
                obstacles=[obstacle(9.19, -3.86), obstacle(130.0, 1.38)],
                collided=False,
                progress=68.82,
                angles=[0.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -7.0],
            ),
        )

        self.assertLessEqual(self.car.target_offset, 3.2)
        self.assertLessEqual(command.steering, -0.18)

    def test_map31_after_left_obstacle_avoids_right_edge_hit(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=130.12,
                to_middle=7.57,
                moving_angle=14.9,
                obstacles=[obstacle(2.75, -3.86), obstacle(130.0, 1.38)],
                collided=False,
                progress=69.09,
                angles=[0.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -7.0],
            ),
        )

        self.assertLessEqual(command.steering, -0.58)
        self.assertEqual(command.throttle, 0.0)
        self.assertGreaterEqual(command.brake, 0.45)

    def test_map31_second_left_obstacle_commits_right_before_contact(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=153.06,
                to_middle=0.05,
                moving_angle=-3.2,
                obstacles=[obstacle(32.25, -3.86), obstacle(160.0, 1.38)],
                collided=False,
                progress=68.28,
                angles=[0.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -7.0],
            ),
        )

        self.assertGreaterEqual(self.car.target_offset, 3.0)
        self.assertGreater(command.steering, 0.10)
        self.assertEqual(command.throttle, 1.0)
        self.assertEqual(command.brake, 0.0)

    def test_finish_section_pulls_off_right_edge_before_guardrail(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=145.08,
                to_middle=6.97,
                moving_angle=14.8,
                obstacles=[obstacle(123.34, -0.98)],
                collided=False,
                progress=99.73,
                angles=[0.0, 1.0, 3.0, 3.0, 3.0, 3.0, 2.0, 1.0],
            ),
        )

        self.assertLessEqual(self.car.target_offset, 4.8)
        self.assertLessEqual(command.steering, -0.34)
        self.assertEqual(command.throttle, 0.0)
        self.assertGreaterEqual(command.brake, 0.35)

    def test_finish_section_catches_rightward_slingshot_after_last_pole(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=123.24,
                to_middle=-1.62,
                moving_angle=20.9,
                obstacles=[obstacle(-0.83, 7.25), obstacle(142.22, -0.98)],
                collided=False,
                progress=98.92,
                angles=[0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 0.0],
            ),
        )

        self.assertLessEqual(command.steering, -0.42)
        self.assertEqual(command.throttle, 0.0)
        self.assertGreaterEqual(command.brake, 0.35)

    def test_late_section_uses_lighter_hard_corner_brake_when_already_slow(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=85.60,
                to_middle=-7.52,
                moving_angle=-10.3,
                obstacles=[obstacle(39.93, 4.87), obstacle(80.0, -2.42)],
                collided=False,
                progress=88.17,
                angles=[0.0, 4.0, 8.0, 55.0, 55.0, 50.0, 35.0, 20.0],
            ),
        )

        self.assertEqual(command.throttle, 0.0)
        self.assertLessEqual(command.brake, 0.35)

    def test_early_obstacle_target_stays_inside_left_edge(self):
        self.car.start_time = datetime.now() - timedelta(seconds=16)

        command = self.car.control_driving(
            controls(),
            sensing(
                speed=96.32,
                to_middle=-7.28,
                moving_angle=-13.4,
                obstacles=[obstacle(8.72, -0.76), obstacle(56.97, 3.76)],
                collided=False,
                progress=8.60,
                angles=[5.0, 6.0, 8.0, 10.0, 12.0, 12.0, 10.0, 8.0],
            ),
        )

        self.assertGreaterEqual(self.car.target_offset, -5.5)
        self.assertGreater(command.steering, 0.25)

    def test_v10_6_high_speed_edge_guard_before_wall_hit(self):
        command = self.car.control_driving(
            controls(),
            sensing(
                speed=145.82,
                to_middle=7.50,
                moving_angle=10.9,
                obstacles=[],
                collided=False,
                progress=49.73,
                angles=[2.0, 3.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0],
            ),
        )

        self.assertLessEqual(command.steering, -0.32)
        self.assertEqual(command.throttle, 0.0)
        self.assertGreaterEqual(command.brake, 0.35)

if __name__ == "__main__":
    unittest.main()
