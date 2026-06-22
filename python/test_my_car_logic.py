import unittest
from types import SimpleNamespace

from my_car import DrivingClient


def state(speed, to_middle, moving_angle, *angles, obstacles=None, lap_progress=1.0):
    return SimpleNamespace(
        speed=speed,
        to_middle=to_middle,
        moving_angle=moving_angle,
        moving_forward=True,
        lap_progress=lap_progress,
        half_road_limit=10.0,
        track_forward_angles=list(angles),
        distance_to_way_points=[10.0 for _ in angles],
        track_forward_obstacles=obstacles or [],
        opponent_cars_info=[],
        collided=False,
    )


def obstacle(dist, to_middle):
    return {"dist": dist, "to_middle": to_middle}


class MyCarSpeedLogicTests(unittest.TestCase):
    def test_straight_road_accelerates_without_steering_noise(self):
        car = DrivingClient()

        command = car.decide_controls(state(80, 0, 0, 0, 0, 0, 0))

        self.assertAlmostEqual(command.steering, 0.0, delta=0.04)
        self.assertAlmostEqual(command.throttle, 1.0)
        self.assertAlmostEqual(command.brake, 0.0)

    def test_player_name_is_registered_for_simulator(self):
        car = DrivingClient()

        self.assertEqual(car.set_player_name(), "Car1")
        car.getPlayerName({"Vehicles": {"Car1": {}}})
        self.assertEqual(car.player_name, "Car1")


    def test_follows_forward_curve_direction(self):
        car = DrivingClient()

        right = car.decide_controls(state(80, 0, 0, 10, 20, 30, 35))
        left = car.decide_controls(state(80, 0, 0, -10, -20, -30, -35))

        self.assertGreater(right.steering, 0.08)
        self.assertLess(left.steering, -0.08)

    def test_recenters_without_reverse_when_off_middle(self):
        car = DrivingClient()

        right = car.decide_controls(state(45, 7.0, 3.0, 0, 0, 0, 0))
        left = car.decide_controls(state(45, -7.0, -3.0, 0, 0, 0, 0))

        self.assertLess(right.steering, -0.20)
        self.assertGreater(left.steering, 0.20)
        self.assertGreaterEqual(right.throttle, 0.0)
        self.assertGreaterEqual(left.throttle, 0.0)
        self.assertAlmostEqual(right.brake, 0.0)
        self.assertAlmostEqual(left.brake, 0.0)

    def test_guardrail_zone_drives_inward_without_collision_reverse(self):
        car = DrivingClient()

        command = car.decide_controls(state(8.0, -15.0, -8.0, 0, 0, 0, 0))

        self.assertGreater(command.steering, 0.65)
        self.assertGreaterEqual(command.throttle, 0.0)
        self.assertAlmostEqual(command.brake, 0.0)
        self.assertFalse(car.is_accident)

    def test_high_speed_yaw_uses_short_brake_before_wall(self):
        car = DrivingClient()

        command = car.decide_controls(state(115.0, -3.0, -58.0, 0, 0, 0, 0))

        self.assertGreater(command.steering, 0.45)
        self.assertLessEqual(command.throttle, 0.25)
        self.assertGreaterEqual(command.brake, 0.20)

    def test_avoids_obstacle_by_moving_to_open_lane(self):
        car = DrivingClient()

        right_obstacle = car.decide_controls(
            state(75, 0, 0, 0, 0, 0, 0, obstacles=[obstacle(18.0, 1.0)])
        )
        left_obstacle = car.decide_controls(
            state(75, 0, 0, 0, 0, 0, 0, obstacles=[obstacle(18.0, -1.0)])
        )

        self.assertLess(right_obstacle.steering, -0.08)
        self.assertGreater(left_obstacle.steering, 0.08)
        self.assertAlmostEqual(right_obstacle.brake, 0.0)
        self.assertAlmostEqual(left_obstacle.brake, 0.0)

    def test_obstacle_avoidance_does_not_steer_farther_out_near_edge(self):
        car = DrivingClient()

        right_edge = car.decide_controls(
            state(80, 4.0, 0, 0, 0, 0, 0, obstacles=[obstacle(18.0, 2.5)])
        )
        left_edge = car.decide_controls(
            state(80, -4.0, 0, 0, 0, 0, 0, obstacles=[obstacle(18.0, -2.5)])
        )

        self.assertLessEqual(right_edge.steering, 0.0)
        self.assertGreaterEqual(left_edge.steering, 0.0)

    def test_keeps_lane_offset_until_center_obstacle_is_passed(self):
        car = DrivingClient()

        command = car.decide_controls(
            state(65, -5.6, 18.0, 0, 0, 0, 0, obstacles=[obstacle(11.0, 0.72)])
        )

        self.assertLess(command.steering, 0.30)
        self.assertLessEqual(command.throttle, 0.35)

    def test_edge_obstacle_target_stays_on_clear_side(self):
        car = DrivingClient()

        left_edge_target = car.choose_lane_target(
            state(62, -7.9, 6.0, 0, 0, 0, 0, obstacles=[obstacle(15.0, 0.72)])
        )
        right_edge_target = car.choose_lane_target(
            state(62, 7.9, -6.0, 0, 0, 0, 0, obstacles=[obstacle(15.0, -0.72)])
        )

        self.assertLess(left_edge_target, -1.5)
        self.assertGreater(right_edge_target, 1.5)

    def test_center_obstacle_is_planned_before_close_range(self):
        car = DrivingClient()

        left_clear_target = car.choose_lane_target(
            state(68, -5.5, -20.0, 0, 0, 0, 0, obstacles=[obstacle(28.0, 0.72)])
        )
        right_clear_target = car.choose_lane_target(
            state(68, 5.5, 20.0, 0, 0, 0, 0, obstacles=[obstacle(28.0, -0.72)])
        )

        self.assertLess(left_clear_target, -1.5)
        self.assertGreater(right_clear_target, 1.5)

    def test_obstacle_side_uses_motion_when_lateral_difference_is_small(self):
        car = DrivingClient()

        moving_right_target = car.choose_lane_target(
            state(107, -1.04, 2.7, 0, 0, 0, 0, obstacles=[obstacle(26.0, -0.76)])
        )
        moving_left_target = car.choose_lane_target(
            state(107, 1.04, -2.7, 0, 0, 0, 0, obstacles=[obstacle(26.0, 0.76)])
        )

        self.assertGreater(moving_right_target, 1.5)
        self.assertLess(moving_left_target, -1.5)

    def test_close_obstacle_caps_steering_toward_collision_side(self):
        car = DrivingClient()

        left_clear = car.decide_controls(
            state(60, -6.5, 18.0, 30, 30, 30, 25, obstacles=[obstacle(10.0, 0.72)])
        )
        right_clear = car.decide_controls(
            state(60, 6.5, -18.0, -30, -30, -30, -25, obstacles=[obstacle(10.0, -0.72)])
        )
        immediate_left_clear = car.decide_controls(
            state(55, -5.8, 27.0, 25, 30, 30, 25, obstacles=[obstacle(7.0, 0.72)])
        )
        immediate_right_clear = car.decide_controls(
            state(55, 5.8, -27.0, -25, -30, -30, -25, obstacles=[obstacle(7.0, -0.72)])
        )

        self.assertLessEqual(left_clear.steering, 0.0)
        self.assertGreaterEqual(right_clear.steering, 0.0)
        self.assertLessEqual(immediate_left_clear.steering, -0.05)
        self.assertGreaterEqual(immediate_right_clear.steering, 0.05)
        self.assertLessEqual(left_clear.throttle, 0.35)
        self.assertLessEqual(right_clear.throttle, 0.35)

    def test_close_obstacle_forces_clearance_when_lateral_gap_is_small(self):
        car = DrivingClient()

        right_clear = car.decide_controls(
            state(56, 6.35, -7.6, -25, -30, -30, -25, obstacles=[obstacle(6.8, 3.76)])
        )
        left_clear = car.decide_controls(
            state(56, -6.35, 7.6, 25, 30, 30, 25, obstacles=[obstacle(6.8, -3.76)])
        )

        self.assertGreaterEqual(right_clear.steering, 0.16)
        self.assertLessEqual(left_clear.steering, -0.16)

    def test_near_edge_overrides_curve_to_return_inward(self):
        car = DrivingClient()

        right = car.decide_controls(state(55, 9.5, 8.0, 25, 30, 30, 25))
        left = car.decide_controls(state(55, -9.5, -8.0, -25, -30, -30, -25))

        self.assertLess(right.steering, -0.25)
        self.assertGreater(left.steering, 0.25)

    def test_stopped_without_collision_keeps_driving_forward(self):
        car = DrivingClient()
        command = None

        for _ in range(10):
            command = car.decide_controls(state(0.2, 0, 0, 0, 0, 0, 0, lap_progress=3.0))

        self.assertGreater(command.throttle, 0.0)
        self.assertAlmostEqual(command.brake, 0.0)
        self.assertFalse(car.is_accident)

    def test_wall_collision_reverses_then_turns_inward(self):
        car = DrivingClient()
        wall_state = state(1.0, -14.5, 0, 0, 0, 0, 0, lap_progress=4.0)
        wall_state.collided = True

        commands = [car.decide_controls(wall_state) for _ in range(9)]

        self.assertTrue(all(command.throttle < 0.0 for command in commands[:5]))
        self.assertTrue(all(command.steering > 0.85 for command in commands[5:]))
        self.assertTrue(all(command.throttle > 0.0 for command in commands[5:]))

    def test_wall_recovery_releases_after_car_moves_clear(self):
        car = DrivingClient()
        car.is_accident = True
        car.collision_mode = "wall"
        car.recovery_count = 8

        command = car.decide_controls(state(20.0, -8.0, 12.0, 0, 0, 0, 0, lap_progress=4.0))

        self.assertGreaterEqual(command.throttle, 0.0)
        self.assertFalse(car.is_accident)


if __name__ == "__main__":
    unittest.main()
