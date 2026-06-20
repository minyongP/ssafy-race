import DrivingInterface.*;

public class MyCar {

    private static final float CENTERING_FACTOR = 80.0f;
    private static final float SPEED_LOOKAHEAD_UNIT = 30.0f;
    private static final float ANGLE_LOOKAHEAD_UNIT = 45.0f;
    private static final float SHARP_CURVE_ANGLE = 45.0f;
    private static final float EMERGENCY_CURVE_ANGLE = 80.0f;
    private static final float EMERGENCY_STEERING_BOOST = 0.30f;
    private static final float OBSTACLE_SCAN_DISTANCE = 30.0f;
    private static final float OBSTACLE_SAFE_WIDTH = 2.2f;
    private static final float OBSTACLE_STEER_COEFF = 50.0f;
    private static final float ROAD_HALF_WIDTH = 5.0f;
    private static final float EPSILON = 0.001f;

    boolean is_accident = false;
    int accident_count = 0;
    int recovery_count = 0;

    boolean is_debug = false;
    static boolean enable_api_control = true; // true(Controlled by code) /false(Controlled by keyboard)

    public void control_driving(boolean a1, float a2, float a3, float a4, float a5, float a6, float a7, float a8,
                                float[] a9, float[] a10, float[] a11, float[] a12) {

        // ===========================================================
        // Don't remove this area. ===================================
        // ===========================================================
        DrivingInterface di = new DrivingInterface();
        DrivingInterface.CarStateValues sensing_info = di.get_car_state(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12);
        // ===========================================================

        if(is_debug) {
            System.out.println("=========================================================");
            System.out.println("[MyCar] to middle: " + sensing_info.to_middle);

            System.out.println("[MyCar] collided: " + sensing_info.collided);
            System.out.println("[MyCar] car speed: " + sensing_info.speed + "km/h");

            System.out.println("[MyCar] is moving forward: " + sensing_info.moving_forward);
            System.out.println("[MyCar] moving angle: " + sensing_info.moving_angle);
            System.out.println("[MyCar] lap_progress: " + sensing_info.lap_progress);

            StringBuilder forward_angles = new StringBuilder("[MyCar] track_forward_angles: ");
            for (Float track_forward_angle : sensing_info.track_forward_angles) {
                forward_angles.append(track_forward_angle).append(", ");
            }
            System.out.println(forward_angles);

            StringBuilder to_way_points = new StringBuilder("[MyCar] distance_to_way_points: ");
            for (Float distance_to_way_point : sensing_info.distance_to_way_points) {
                to_way_points.append(distance_to_way_point).append(", ");
            }
            System.out.println(to_way_points);

            StringBuilder forward_obstacles = new StringBuilder("[MyCar] track_forward_obstacles: ");
            for (DrivingInterface.ObstaclesInfo track_forward_obstacle : sensing_info.track_forward_obstacles) {
                forward_obstacles.append("{dist:").append(track_forward_obstacle.dist)
                        .append(", to_middle:").append(track_forward_obstacle.to_middle).append("}, ");
            }
            System.out.println(forward_obstacles);

            StringBuilder opponent_cars = new StringBuilder("[MyCar] opponent_cars_info: ");
            for (DrivingInterface.CarsInfo carsInfo : sensing_info.opponent_cars_info) {
                opponent_cars.append("{dist:").append(carsInfo.dist)
                        .append(", to_middle:").append(carsInfo.to_middle)
                        .append(", speed:").append(carsInfo.speed).append("km/h}, ");
            }
            System.out.println(opponent_cars);

            System.out.println("=========================================================");
        }

        // ===========================================================
        // Area for writing code about driving rule ==================
        // ===========================================================
        // Editing area starts from here
        //

        DriveCommand command = decideControls(sensing_info);
        car_controls.steering = command.steering;
        car_controls.throttle = command.throttle;
        car_controls.brake = command.brake;

        if(is_debug) {
            System.out.println("[MyCar] steering:"+car_controls.steering+
                                     ", throttle:"+car_controls.throttle+", brake:"+car_controls.brake);
        }

        //
        // Editing area ends
        // =======================================================
    }

    static class DriveCommand {
        final float steering;
        final float throttle;
        final float brake;

        DriveCommand(float steering, float throttle, float brake) {
            this.steering = steering;
            this.throttle = throttle;
            this.brake = brake;
        }
    }

    static class TrackRisk {
        final boolean fullThrottle;
        final boolean emergencyCurve;

        TrackRisk(boolean fullThrottle, boolean emergencyCurve) {
            this.fullThrottle = fullThrottle;
            this.emergencyCurve = emergencyCurve;
        }
    }

    DriveCommand decideControls(DrivingInterface.CarStateValues sensing_info) {
        DriveCommand recovery_command = getRecoveryCommand(sensing_info);
        if (recovery_command != null) {
            return recovery_command;
        }

        float ref_angle = getReferenceAngle(sensing_info);
        TrackRisk track_risk = inspectForwardTrack(sensing_info);
        float steering = calculateBaseSteering(sensing_info, ref_angle);
        DriveCommand speed_command = calculateSpeedCommand(sensing_info.speed, ref_angle, track_risk);

        if (track_risk.emergencyCurve) {
            steering = addEmergencySteering(steering);
        }

        DriveCommand command = avoidObstacle(
                sensing_info,
                steering,
                speed_command.throttle,
                speed_command.brake
        );
        return clampCommand(command);
    }

    private DriveCommand getRecoveryCommand(DrivingInterface.CarStateValues sensing_info) {
        if (sensing_info.speed > 30.0f) {
            is_accident = false;
            recovery_count = 0;
            accident_count = 0;
            return null;
        }

        if (sensing_info.lap_progress > 0.5f && !is_accident && Math.abs(sensing_info.speed) < 1.0f) {
            accident_count++;
        }

        if (accident_count > 6) {
            is_accident = true;
        }

        if (!is_accident) {
            return null;
        }

        recovery_count++;
        if (recovery_count > 20) {
            is_accident = false;
            recovery_count = 0;
            accident_count = 0;
            return new DriveCommand(0.0f, 0.0f, 0.0f);
        }

        return new DriveCommand(0.02f, -1.0f, 0.0f);
    }

    private TrackRisk inspectForwardTrack(DrivingInterface.CarStateValues sensing_info) {
        boolean full_throttle = true;
        boolean emergency_curve = false;
        int road_range = Math.min(
                sensing_info.track_forward_angles.size(),
                Math.max(0, (int)(sensing_info.speed / SPEED_LOOKAHEAD_UNIT))
        );

        for (int i = 0; i < road_range; i++) {
            float forward_angle = Math.abs(sensing_info.track_forward_angles.get(i));
            if (forward_angle > SHARP_CURVE_ANGLE) {
                full_throttle = false;
            }
            if (forward_angle > EMERGENCY_CURVE_ANGLE) {
                emergency_curve = true;
                break;
            }
        }

        return new TrackRisk(full_throttle, emergency_curve);
    }

    private float calculateBaseSteering(DrivingInterface.CarStateValues sensing_info, float ref_angle) {
        float center_correction = (sensing_info.to_middle / CENTERING_FACTOR) * -1.0f;
        float steer_factor = getSteerFactor(sensing_info.speed) + EPSILON;
        return ((ref_angle - sensing_info.moving_angle) / steer_factor) + center_correction;
    }

    private DriveCommand calculateSpeedCommand(float speed, float ref_angle, TrackRisk track_risk) {
        float throttle = getBaseThrottle(speed, ref_angle);
        float brake = 0.0f;

        if (!track_risk.fullThrottle) {
            if (speed > 100.0f) {
                brake = 0.30f;
            }
            if (speed > 120.0f) {
                throttle = 0.70f;
                brake = 0.70f;
            }
            if (speed > 130.0f) {
                throttle = 0.50f;
                brake = 1.0f;
            }
        }

        return new DriveCommand(0.0f, throttle, brake);
    }

    private float addEmergencySteering(float steering) {
        return steering + (steering > 0.0f ? EMERGENCY_STEERING_BOOST : -EMERGENCY_STEERING_BOOST);
    }

    private DriveCommand avoidObstacle(
            DrivingInterface.CarStateValues sensing_info,
            float steering,
            float throttle,
            float brake
    ) {
        DrivingInterface.ObstaclesInfo obstacle = getFirstObstacle(sensing_info);
        if (obstacle == null || obstacle.dist >= OBSTACLE_SCAN_DISTANCE) {
            return new DriveCommand(steering, throttle, brake);
        }

        float obstacle_to_middle = obstacle.to_middle;
        float car_to_middle = sensing_info.to_middle;
        float diff_to_middle = obstacle_to_middle - car_to_middle;
        if (Math.abs(diff_to_middle) >= OBSTACLE_SAFE_WIDTH) {
            return new DriveCommand(steering, throttle, brake);
        }

        float avoid_strength = calculateAvoidStrength(sensing_info.speed, diff_to_middle);
        float avoid_direction = calculateAvoidDirection(car_to_middle, obstacle_to_middle);
        steering = avoid_direction * avoid_strength;

        return new DriveCommand(steering, throttle, brake);
    }

    private float calculateAvoidStrength(float speed, float diff_to_middle) {
        float need_steering = (OBSTACLE_SAFE_WIDTH - Math.abs(diff_to_middle)) / OBSTACLE_SAFE_WIDTH;
        return need_steering * OBSTACLE_STEER_COEFF / (getSteerFactor(speed) + EPSILON);
    }

    private float calculateAvoidDirection(float car_to_middle, float obstacle_to_middle) {
        boolean obstacle_is_right = obstacle_to_middle >= car_to_middle;
        boolean can_move_left = ROAD_HALF_WIDTH + Math.min(car_to_middle, obstacle_to_middle) > OBSTACLE_SAFE_WIDTH;
        boolean can_move_right = ROAD_HALF_WIDTH - Math.max(car_to_middle, obstacle_to_middle) > OBSTACLE_SAFE_WIDTH;

        if (obstacle_is_right) {
            return can_move_left ? -1.0f : 1.0f;
        }
        return can_move_right ? 1.0f : -1.0f;
    }

    private DrivingInterface.ObstaclesInfo getFirstObstacle(DrivingInterface.CarStateValues sensing_info) {
        if (sensing_info.track_forward_obstacles.isEmpty()) {
            return null;
        }
        return sensing_info.track_forward_obstacles.get(0);
    }

    private float getReferenceAngle(DrivingInterface.CarStateValues sensing_info) {
        if (sensing_info.track_forward_angles.isEmpty()) {
            return 0.0f;
        }

        int angle_index = Math.min(
                sensing_info.track_forward_angles.size() - 1,
                Math.max(0, (int)(sensing_info.speed / ANGLE_LOOKAHEAD_UNIT))
        );
        return sensing_info.track_forward_angles.get(angle_index);
    }

    private float getSteerFactor(float speed) {
        float steer_factor = speed * 1.50f;
        if (speed > 70.0f) {
            steer_factor = speed * 0.85f;
        }
        if (speed > 100.0f) {
            steer_factor = speed * 0.70f;
        }
        return steer_factor;
    }

    private float getBaseThrottle(float speed, float ref_angle) {
        float throttle_factor = 0.6f / (Math.abs(ref_angle) + 0.1f);
        if (throttle_factor > 0.11f) {
            throttle_factor = 0.11f;
        }

        float throttle = 0.7f + throttle_factor;
        if (speed < 60.0f) {
            throttle = 0.9f;
        }

        return throttle;
    }

    private float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    private DriveCommand clampCommand(DriveCommand command) {
        return new DriveCommand(
                clamp(command.steering, -1.0f, 1.0f),
                clamp(command.throttle, -1.0f, 1.0f),
                clamp(command.brake, 0.0f, 1.0f)
        );
    }

    // ===========================================================
    // Don't remove below area. ==================================
    // ===========================================================
    public native int StartDriving(boolean enable_api_control);

    static MyCar car_controls;

    float throttle;
    float steering;
    float brake;

    static {
        System.loadLibrary("DrivingInterface/DrivingInterface");
    }

    public static void main(String[] args) {
        System.out.println("[MyCar] Start Bot! (JAVA)");

        car_controls = new MyCar();
        int return_code = car_controls.StartDriving(enable_api_control);

        System.out.println("[MyCar] End Bot! (JAVA), return_code = " + return_code);

        System.exit(return_code);
    }
    // ===========================================================
}
