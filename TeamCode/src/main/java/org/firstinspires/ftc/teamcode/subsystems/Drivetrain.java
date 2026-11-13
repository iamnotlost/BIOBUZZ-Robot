package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Mecanum drive for TeleOp (the autos drive with Pedro Pathing).
 * Stick feel: dead zone, then any push starts at the smallest power that moves the robot and curves up to the
 * top speed. Speeding up takes RAMP_SECONDS, slowing down STOP_SECONDS (motors brake at 0).
 */
public class Drivetrain {
    // Top speeds (0-1). MAX_SPEED: right stick click / gamepad 2 bumpers, saved in Settings
    public static double MAX_SPEED  = 0.4;
    public static double TURN_SPEED = 0.4;
    public static final double SPEED_STEP = 0.1;
    public static final double[] SPEED_LEVELS = {0.4, 0.6, 0.8, 1.0};

    // 0.2 s to full speed (0.5 felt slow to react), 0.1 s to stop (stopping at once felt jerky)
    public static double RAMP_SECONDS = 0.2;
    public static double STOP_SECONDS = 0.1;

    // Stick feel. Plain squaring left the first third of the stick dead; this starts every push at the smallest
    // power that really moves the robot.
    public static double STICK_DEAD_ZONE = 0.06;
    public static double MIN_DRIVE_POWER = 0.07;
    public static double MIN_TURN_POWER  = 0.06;
    public static double CURVE_POWER     = 1.8;
    private static final double STRAFE_CORRECTION = 1.1;

    private final RobotHardware hw;
    private double lastX, lastY, lastRx;    // last command (after the ramp)
    private final ElapsedTime loopTimer = new ElapsedTime();

    public Drivetrain(RobotHardware hw) {
        this.hw = hw;
    }

    /** Stick amount (0..1, or -1..1 for turning) -> power between the smallest moving power and {@code top}. */
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

    /** Next of {@link #SPEED_LEVELS}, back to the slowest after 100%. Returns its number (1-4). */
    public static int nextSpeedLevel() {
        int next = 0;
        for (int i = 0; i < SPEED_LEVELS.length; i++) {
            if (SPEED_LEVELS[i] > MAX_SPEED + 0.01) { next = i; break; }
        }
        MAX_SPEED = SPEED_LEVELS[next];
        return next + 1;
    }

    /**
     * Drives: x = left/right, y = forward/back (stick directions), rx = turn.
     * {@code fieldHeading}: robot heading for field-centric driving, 0 for robot-centric.
     */
    public void drive(double x, double y, double rx, double fieldHeading) {
        rampTo(x, y, rx);
        double cos = Math.cos(-fieldHeading), sin = Math.sin(-fieldHeading);
        setPowers(lastX * cos - lastY * sin, lastX * sin + lastY * cos, lastRx);
    }

    /** Stops at once, without the ramp. */
    public void stop() {
        lastX = lastY = lastRx = 0;
        loopTimer.reset();
        setPowers(0, 0, 0);
    }

    // Moves the last command toward the target by at most one time step. Driving ramps as one arrow, so the
    // robot always heads exactly where the stick points and only the speed eases in.
    private void rampTo(double x, double y, double rx) {
        double dt = Math.min(loopTimer.seconds(), 0.1);   // a stalled loop must not cause a jump
        loopTimer.reset();
        double up = RAMP_SECONDS > 0 ? dt / RAMP_SECONDS : 1;
        double down = dt / STOP_SECONDS;

        double dx = x - lastX, dy = y - lastY, dist = Math.hypot(dx, dy);
        double step = Math.hypot(x, y) < Math.hypot(lastX, lastY) ? down : up;
        if (dist <= step) {
            lastX = x;
            lastY = y;
        } else {
            lastX += dx / dist * step;
            lastY += dy / dist * step;
        }

        double dr = rx - lastRx;
        double turnStep = Math.abs(rx) < Math.abs(lastRx) ? down : up;
        lastRx = Math.abs(dr) <= turnStep ? rx : lastRx + Math.signum(dr) * turnStep;
    }

    private void setPowers(double x, double y, double rx) {
        x *= STRAFE_CORRECTION;
        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        // scale down only if a wheel would pass full power, all wheels equally, so turning keeps the drive speed
        double max = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));
        setPower(hw.frontLeft, fl / max);
        setPower(hw.backLeft, bl / max);
        setPower(hw.frontRight, fr / max);
        setPower(hw.backRight, br / max);
    }

    private static void setPower(DcMotorEx motor, double power) {
        if (motor != null) motor.setPower(power);
    }
}
