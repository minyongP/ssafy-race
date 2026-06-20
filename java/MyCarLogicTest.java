import DrivingInterface.*;

public class MyCarLogicTest {
    public static void main(String[] args) {
        turnsTowardForwardCurve();
        recentersWhenCarIsOffMiddle();
        slowsBeforeSharpFastCurve();
        keepsStrongThrottleBelowSixty();
        avoidsObstacleOnRightBySteeringLeft();
        avoidsObstacleOnLeftBySteeringRight();
        avoidsRightObstacleFromRightSideBySteeringLeft();
        avoidsLeftObstacleFromLeftSideBySteeringRight();
        recoversWhenStoppedAfterStart();
        clampsSteeringRange();
        System.out.println("MyCarLogicTest passed");
    }

    static void turnsTowardForwardCurve() {
        MyCar car = new MyCar();
        MyCar.DriveCommand right = car.decideControls(state(80, 0, 0, 8, 20, 25, 20));
        MyCar.DriveCommand left = car.decideControls(state(80, 0, 0, -8, -20, -25, -20));

        assertTrue(right.steering > 0.05f, "right curve should steer right");
        assertTrue(left.steering < -0.05f, "left curve should steer left");
    }

    static void recentersWhenCarIsOffMiddle() {
        MyCar car = new MyCar();
        MyCar.DriveCommand command = car.decideControls(state(60, 20, 0, 0, 0, 0, 0));

        assertTrue(command.steering < -0.1f, "positive to_middle should steer left toward center");
    }

    static void slowsBeforeSharpFastCurve() {
        MyCar car = new MyCar();
        MyCar.DriveCommand command = car.decideControls(state(125, 0, 0, 90, 90, 90, 90));

        assertTrue(command.throttle <= 0.7f, "sharp fast curve should lower throttle");
        assertTrue(command.brake >= 0.6f, "sharp fast curve should apply brake");
    }

    static void keepsStrongThrottleBelowSixty() {
        MyCar car = new MyCar();
        MyCar.DriveCommand command = car.decideControls(state(55, 0, 0, 20, 20, 20, 20));

        assertClose(0.9f, command.throttle, 0.0001f, "speed below 60 should keep Python baseline throttle");
    }

    static void avoidsObstacleOnRightBySteeringLeft() {
        MyCar car = new MyCar();
        DrivingInterface.CarStateValues values = state(80, 0, 0, 0, 0, 0, 0);
        addObstacle(values, 18.0f, 1.0f);

        MyCar.DriveCommand command = car.decideControls(values);

        assertTrue(command.steering < -0.10f, "right-side obstacle should steer left");
    }

    static void avoidsObstacleOnLeftBySteeringRight() {
        MyCar car = new MyCar();
        DrivingInterface.CarStateValues values = state(80, 0, 0, 0, 0, 0, 0);
        addObstacle(values, 18.0f, -1.0f);

        MyCar.DriveCommand command = car.decideControls(values);

        assertTrue(command.steering > 0.10f, "left-side obstacle should steer right");
    }

    static void avoidsRightObstacleFromRightSideBySteeringLeft() {
        MyCar car = new MyCar();
        DrivingInterface.CarStateValues values = state(80, 1.0f, 0, 0, 0, 0, 0);
        addObstacle(values, 18.0f, 1.5f);

        MyCar.DriveCommand command = car.decideControls(values);

        assertTrue(command.steering < -0.10f, "right-side obstacle while right of center should steer left");
    }

    static void avoidsLeftObstacleFromLeftSideBySteeringRight() {
        MyCar car = new MyCar();
        DrivingInterface.CarStateValues values = state(80, -1.0f, 0, 0, 0, 0, 0);
        addObstacle(values, 18.0f, -1.5f);

        MyCar.DriveCommand command = car.decideControls(values);

        assertTrue(command.steering > 0.10f, "left-side obstacle while left of center should steer right");
    }

    static void recoversWhenStoppedAfterStart() {
        MyCar car = new MyCar();
        MyCar.DriveCommand command = null;
        for (int i = 0; i < 8; i++) {
            command = car.decideControls(state(0.2f, 0, 0, 0, 0, 0, 0, 0));
        }

        assertTrue(command.throttle < 0.0f, "stopped car should reverse for recovery");
        assertClose(0.0f, command.brake, 0.0001f, "recovery should release brake");
    }

    static void clampsSteeringRange() {
        MyCar car = new MyCar();
        MyCar.DriveCommand command = car.decideControls(state(30, -80, -80, 120, 120, 120, 120));

        assertTrue(command.steering <= 1.0f, "steering should be clamped to max 1.0");
        assertTrue(command.steering >= -1.0f, "steering should be clamped to min -1.0");
    }

    static DrivingInterface.CarStateValues state(float speed, float toMiddle, float movingAngle, float... angles) {
        DrivingInterface.CarStateValues values = new DrivingInterface.CarStateValues();
        values.speed = speed;
        values.to_middle = toMiddle;
        values.moving_angle = movingAngle;
        values.moving_forward = 1.0f;
        values.lap_progress = 1.0f;
        values.half_road_limit = 10.0f;
        for (float angle : angles) {
            values.track_forward_angles.add(angle);
            values.distance_to_way_points.add(10.0f);
        }
        return values;
    }

    static void addObstacle(DrivingInterface.CarStateValues values, float dist, float toMiddle) {
        DrivingInterface.ObstaclesInfo obstacle = new DrivingInterface.ObstaclesInfo();
        obstacle.dist = dist;
        obstacle.to_middle = toMiddle;
        values.track_forward_obstacles.add(obstacle);
    }

    static void assertTrue(boolean value, String message) {
        if (!value) {
            throw new AssertionError(message);
        }
    }

    static void assertClose(float expected, float actual, float tolerance, String message) {
        if (Math.abs(expected - actual) > tolerance) {
            throw new AssertionError(message + " expected=" + expected + " actual=" + actual);
        }
    }
}
