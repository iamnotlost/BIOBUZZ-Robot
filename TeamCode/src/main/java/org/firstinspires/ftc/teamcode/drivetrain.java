package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class drivetrain {
    hwManager hwManager;
    private static final double STRAFE_CORRECTION = 1.1;

    // Top speeds (0-1) for TeleOp sticks. MAX_SPEED is adjustable while driving (gamepad 1 D-pad up/down)
    public static double MAX_SPEED  = 0.4;
    public static double TURN_SPEED = 0.4;
    public static final double SPEED_STEP = 0.1;

    // Seconds to go from stopped to full power: short, so the robot reacts at once but doesn't jerk.
    // (0.5 felt slow to react on 2026-11-12.)
    public static double RAMP_SECONDS = 0.2;
    // Seconds to go from full power to stopped when you slow down or let go: quick enough that the robot
    // doesn't keep rolling, smooth enough that it doesn't jerk (stopping at once felt jerky). Motors brake at 0.
    public static double STOP_SECONDS = 0.1;

    // Last command sent: driving as one arrow (x, y) plus turning
    private double lastX, lastY, lastRx;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public drivetrain(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    public void resetYaw() {
        hwManager.resetHeading();
    }

    // Sticks closer to center than this count as zero, so a light touch or a worn stick doesn't creep
    public static double STICK_DEAD_ZONE = 0.06;

    // Dead zone, then squared so small pushes are gentle while a full push still reaches 1
    public static double curve(double stick) {
        double a = Math.abs(stick);
        if (a < STICK_DEAD_ZONE) return 0;
        double scaled = (a - STICK_DEAD_ZONE) / (1 - STICK_DEAD_ZONE);
        return Math.signum(stick) * scaled * scaled;
    }

    // TeleOp stick feel. Any push past the dead zone starts at the smallest power that really moves the
    // robot (below that the wheels just hum), then curves up to the top speed: small pushes = small,
    // controllable moves; a full push = top speed. (Plain squaring left the first third of the stick dead.)
    public static double MIN_DRIVE_POWER = 0.07;
    public static double MIN_TURN_POWER  = 0.06;
    public static double CURVE_POWER     = 1.8;

    // stick amount (0..1, or -1..1 for turning) -> power, between the smallest moving power and top
    public static double shape(double stick, double top, double min) {
        double a = Math.min(1, Math.abs(stick));
        if (a < STICK_DEAD_ZONE) return 0;
        double s = (a - STICK_DEAD_ZONE) / (1 - STICK_DEAD_ZONE);
        min = Math.min(min, top);
        return Math.signum(stick) * (min + (top - min) * Math.pow(s, CURVE_POWER));
    }

    public static void adjustMaxSpeed(int steps) {
        MAX_SPEED = Math.max(0.2, Math.min(1.0, MAX_SPEED + steps * SPEED_STEP));
    }

    // Field-centric: x/y are relative to the field, rx turns the robot
    public void drive(double x, double y, double rx) {
        rampTo(x, y, rx);

        double botHeading = hwManager.getHeading();
        double rotX = lastX * Math.cos(-botHeading) - lastY * Math.sin(-botHeading);
        double rotY = lastX * Math.sin(-botHeading) + lastY * Math.cos(-botHeading);

        setPowers(rotX, rotY, lastRx);
    }

    // Robot-centric: forward is wherever the robot faces
    public void driveRobotCentric(double x, double y, double rx) {
        rampTo(x, y, rx);
        setPowers(lastX, lastY, lastRx);
    }

    // Stops at once, without the ramp
    public void stop() {
        lastX = lastY = lastRx = 0;
        loopTimer.reset();
        setPowers(0, 0, 0);
    }

    // Moves the last command toward the target by at most one time-step of ramp.
    // Driving ramps as a single arrow, so the robot always heads exactly where the stick points
    // and only the speed eases in.
    private void rampTo(double x, double y, double rx) {
        double dt = Math.min(loopTimer.seconds(), 0.1); // a stalled loop must not cause a jump
        loopTimer.reset();
        double step = RAMP_SECONDS > 0 ? dt / RAMP_SECONDS : 1;   // speeding up
        double stopStep = dt / STOP_SECONDS;                         // slowing down / stopping

        double dx = x - lastX, dy = y - lastY;
        double dist = Math.hypot(dx, dy);
        double moveStep = Math.hypot(x, y) < Math.hypot(lastX, lastY) ? stopStep : step;
        if (dist <= moveStep) {
            lastX = x;
            lastY = y;
        } else {
            lastX += dx / dist * moveStep;
            lastY += dy / dist * moveStep;
        }

        double dr = rx - lastRx;
        double turnStep = Math.abs(rx) < Math.abs(lastRx) ? stopStep : step;
        lastRx = Math.abs(dr) <= turnStep ? rx : lastRx + Math.signum(dr) * turnStep;
    }

    private void setPowers(double x, double y, double rx) {
        x = x * STRAFE_CORRECTION;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        // Only scale down when a wheel would go past full power, and then all wheels equally,
        // so turning while driving keeps the driving speed instead of cutting it
        double max = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));
        setPower(hwManager.frontLeft,  fl / max);
        setPower(hwManager.backLeft,   bl / max);
        setPower(hwManager.frontRight, fr / max);
        setPower(hwManager.backRight,  br / max);
    }

    private void setPower(DcMotorEx motor, double power) {
        if (motor != null) motor.setPower(power);
    }
}
