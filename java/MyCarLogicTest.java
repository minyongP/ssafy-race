import DrivingInterface.*;

public class MyCarLogicTest {
    public static void main(String[] args) {
        turnsTowardForwardCurve();
        recentersWhenCarIsOffMiddle();
        slowsBeforeSharpFastCurve();
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
        values.half_road_limit = 10.0f;
        for (float angle : angles) {
            values.track_forward_angles.add(angle);
            values.distance_to_way_points.add(10.0f);
        }
        return values;
    }

    static void assertTrue(boolean value, String message) {
        if (!value) {
            throw new AssertionError(message);
        }
    }
}
