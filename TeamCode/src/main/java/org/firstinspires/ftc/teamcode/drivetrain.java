package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class drivetrain {
    hwManager hwManager;
    private static final double STRAFE_CORRECTION = 1.1;

    // Top speeds (0-1) for TeleOp sticks. MAX_SPEED is adjustable while driving (gamepad 1 D-pad up/down)
    public static double MAX_SPEED  = 0.4;
    public static double TURN_SPEED = 0.3;
    public static final double SPEED_STEP = 0.1;
    // Largest change in power per loop, so the robot eases into moves instead of jerking.
    // About 0.03 = full power in ~33 loops (~0.6 s)
    public static double RAMP_PER_LOOP = 0.03;

    private double lastX, lastY, lastRx;

    public drivetrain(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    public void resetYaw() {
        hwManager.resetHeading();
    }

    // Squares the stick so small pushes are gentle while a full push still reaches 1
    public static double curve(double stick) {
        return stick * Math.abs(stick);
    }

    public static void adjustMaxSpeed(int steps) {
        MAX_SPEED = Math.max(0.2, Math.min(1.0, MAX_SPEED + steps * SPEED_STEP));
    }

    // Field-centric: x/y are relative to the field, rx turns the robot
    public void drive(double x, double y, double rx) {
        x = ramp(lastX, x);
        y = ramp(lastY, y);
        rx = ramp(lastRx, rx);
        lastX = x;
        lastY = y;
        lastRx = rx;

        double botHeading = hwManager.getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        setPowers(rotX, rotY, rx);
    }

    // Robot-centric: forward is wherever the robot faces
    public void driveRobotCentric(double x, double y, double rx) {
        x = ramp(lastX, x);
        y = ramp(lastY, y);
        rx = ramp(lastRx, rx);
        lastX = x;
        lastY = y;
        lastRx = rx;

        setPowers(x, y, rx);
    }

    // Stops at once, without the ramp
    public void stop() {
        lastX = lastY = lastRx = 0;
        setPowers(0, 0, 0);
    }

    private static double ramp(double last, double target) {
        double delta = target - last;
        if (Math.abs(delta) <= RAMP_PER_LOOP) return target;
        return last + Math.signum(delta) * RAMP_PER_LOOP;
    }

    private void setPowers(double x, double y, double rx) {
        x = x * STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setPower(hwManager.frontLeft,  (y + x + rx) / denominator);
        setPower(hwManager.backLeft,   (y - x + rx) / denominator);
        setPower(hwManager.frontRight, (y - x - rx) / denominator);
        setPower(hwManager.backRight,  (y + x - rx) / denominator);
    }

    private void setPower(DcMotorEx motor, double power) {
        if (motor != null) motor.setPower(power);
    }
}
