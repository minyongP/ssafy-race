import DrivingInterface.*;

public class MyCar {

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

    DriveCommand decideControls(DrivingInterface.CarStateValues sensing_info) {
        DriveCommand recovery_command = getRecoveryCommand(sensing_info);
        if (recovery_command != null) {
            return recovery_command;
        }

        float ref_angle = getReferenceAngle(sensing_info);
        float middle_add = (sensing_info.to_middle / 70.0f) * -1.0f;
        float steer_factor = getSteerFactor(sensing_info.speed);

        float steering = ((ref_angle - sensing_info.moving_angle) / steer_factor) + middle_add;

        boolean full_throttle = true;
        boolean emergency_brake = false;
        int road_range = Math.min(
                sensing_info.track_forward_angles.size(),
                Math.max(1, (int)(sensing_info.speed / 30.0f))
        );

        for (int i = 0; i < road_range; i++) {
            float forward_angle = Math.abs(sensing_info.track_forward_angles.get(i));
            if (forward_angle > 45.0f) {
                full_throttle = false;
            }
            if (forward_angle > 80.0f) {
                emergency_brake = true;
                break;
            }
        }

        float throttle = getBaseThrottle(sensing_info.speed, ref_angle);
        float brake = 0.0f;

        if (!full_throttle) {
            if (sensing_info.speed > 90.0f) {
                brake = 0.25f;
            }
            if (sensing_info.speed > 115.0f) {
                throttle = Math.min(throttle, 0.65f);
                brake = 0.60f;
            }
            if (sensing_info.speed > 130.0f) {
                throttle = Math.min(throttle, 0.45f);
                brake = 1.0f;
            }
        }

        if (emergency_brake) {
            throttle = Math.min(throttle, 0.55f);
            brake = Math.max(brake, sensing_info.speed > 100.0f ? 0.70f : 0.35f);
            steering += steering >= 0.0f ? 0.20f : -0.20f;
        }

        DriveCommand obstacle_command = avoidObstacle(sensing_info, steering, throttle, brake);
        steering = obstacle_command.steering;
        throttle = obstacle_command.throttle;
        brake = obstacle_command.brake;

        return new DriveCommand(clamp(steering, -1.0f, 1.0f), clamp(throttle, -1.0f, 1.0f), clamp(brake, 0.0f, 1.0f));
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
            return null;
        }

        return new DriveCommand(0.02f, -1.0f, 0.0f);
    }

    private DriveCommand avoidObstacle(
            DrivingInterface.CarStateValues sensing_info,
            float steering,
            float throttle,
            float brake
    ) {
        DrivingInterface.ObstaclesInfo obstacle = getClosestObstacle(sensing_info);
        if (obstacle == null || obstacle.dist > 35.0f) {
            return new DriveCommand(steering, throttle, brake);
        }

        float gap = obstacle.to_middle - sensing_info.to_middle;
        float safe_width = 2.7f;
        if (Math.abs(gap) > safe_width) {
            return new DriveCommand(steering, throttle, brake);
        }

        float obstacle_direction = gap >= 0.0f ? 1.0f : -1.0f;
        float avoid_direction = -obstacle_direction;
        float distance_factor = clamp((35.0f - obstacle.dist) / 35.0f, 0.0f, 1.0f);
        float overlap_factor = clamp((safe_width - Math.abs(gap)) / safe_width, 0.0f, 1.0f);
        float avoid_strength = 0.18f + (0.65f * Math.max(distance_factor, overlap_factor));

        steering += avoid_direction * avoid_strength;

        if (obstacle.dist < 18.0f && sensing_info.speed > 85.0f) {
            throttle = Math.min(throttle, 0.65f);
            brake = Math.max(brake, 0.20f);
        }

        return new DriveCommand(steering, throttle, brake);
    }

    private DrivingInterface.ObstaclesInfo getClosestObstacle(DrivingInterface.CarStateValues sensing_info) {
        DrivingInterface.ObstaclesInfo closest = null;
        for (DrivingInterface.ObstaclesInfo obstacle : sensing_info.track_forward_obstacles) {
            if (obstacle.dist <= 0.0f) {
                continue;
            }
            if (closest == null || obstacle.dist < closest.dist) {
                closest = obstacle;
            }
        }
        return closest;
    }

    private float getReferenceAngle(DrivingInterface.CarStateValues sensing_info) {
        if (sensing_info.track_forward_angles.isEmpty()) {
            return 0.0f;
        }

        int angle_index = Math.min(
                sensing_info.track_forward_angles.size() - 1,
                Math.max(0, (int)(sensing_info.speed / 45.0f))
        );
        return sensing_info.track_forward_angles.get(angle_index);
    }

    private float getSteerFactor(float speed) {
        float steer_factor = Math.max(35.0f, speed * 1.20f);
        if (speed > 70.0f) {
            steer_factor = speed * 0.90f;
        }
        if (speed > 100.0f) {
            steer_factor = speed * 0.75f;
        }
        return Math.max(35.0f, steer_factor);
    }

    private float getBaseThrottle(float speed, float ref_angle) {
        float abs_angle = Math.abs(ref_angle);
        float throttle;

        if (abs_angle < 10.0f) {
            throttle = 0.92f;
        } else if (abs_angle < 30.0f) {
            throttle = 0.82f;
        } else if (abs_angle < 55.0f) {
            throttle = 0.72f;
        } else {
            throttle = 0.62f;
        }

        if (speed < 50.0f) {
            throttle = 1.0f;
        }
        if (speed > 110.0f) {
            throttle = Math.min(throttle, 0.65f);
        }
        if (speed > 130.0f) {
            throttle = Math.min(throttle, 0.50f);
        }

        return throttle;
    }

    private float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
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
