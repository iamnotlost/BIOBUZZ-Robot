package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Speed control for the shooter wheel: both motors get the SAME power from one controller (so they never fight),
 * the speed is read from the shooter motor's encoder. Call {@link #update()} once per loop.
 *
 * <pre>
 * power = kF * target      what it takes to hold the speed           (feedforward)
 *       + kP * error       push harder when too slow, less when fast  (proportional)
 *       + learned extra    fixes the last bit of error, e.g. battery  (integral)
 * </pre>
 * Tuned from the robot logs:
 * - kP 0.001: stronger made the wheel overshoot after every ball (this wheel speeds up very fast).
 * - The integral learns only while the wheel is STEADY and no ball is being fed. Learning while it was still
 *   speeding up ("windup") made it hold 20-30 above the target and creep higher every burst (1190 -> 1240+).
 * - Gentle start (SPINUP_RAMP_PER_S): two motors jumping to 100% from standstill dropped the battery to ~7 V
 *   and the hubs cut the motors.
 * No derivative (D) term: the speed reading comes in steps of 20, so D would make the power jitter.
 */
public class Shooter {
    public static double kF = 0.00037;           // power per tick/s (held 1330 with 0.49 power)
    public static double kP = 0.001;             // power per tick/s of error
    public static double kI = 0.003;             // power per (tick/s of error x second)
    private static final double I_MAX = 0.15;    // the learned extra never goes past this
    private static final double I_ZONE = 150;    // only learn when this close to the target
    public static double STEADY_BAND = 20;       // "steady" = reading changed by at most this in the last 0.1 s

    public static double SPINUP_RAMP_PER_S = 2.0;   // below SPINUP_BELOW of the target: 0 -> 100% in 0.5 s
    public static double SPINUP_BELOW = 0.8;

    // Backwards (hold circle): gentle, and only once the wheel has slowed down - reversing a fast wheel would
    // pull a huge current from the battery
    public static double REVERSE_POWER = 0.3;
    public static double REVERSE_BELOW = 300;    // ticks/s

    private final RobotHardware hw;
    private double target = 0;
    private double integral = 0;
    private boolean learning = true;
    private boolean reversing = false;
    private double power = 0;
    private final ElapsedTime dtTimer = new ElapsedTime();

    public Shooter(RobotHardware hw) {
        this.hw = hw;
    }

    /** Wanted wheel speed in ticks/s; calling it every loop is fine. */
    public void spinTo(int speed) {
        target = Math.max(0, speed);
        reversing = false;
    }

    public void turnOff() {
        target = 0;
        reversing = false;
        setPower(0);
    }

    /** Wheel backwards, so a stuck ball comes back out. */
    public void reverse() {
        target = 0;
        reversing = true;
    }

    /** false while a ball is being fed, so a ball loading the wheel doesn't change the learned extra. */
    public void setLearning(boolean on) {
        learning = on;
    }

    public double power() {
        return power;
    }

    public void update() {
        double dt = Math.min(dtTimer.seconds(), 0.1);
        dtTimer.reset();
        if (hw.shooter == null) return;
        if (target <= 0 && !reversing) {
            setPower(0);          // off: nothing to read
            return;
        }
        double vel = hw.shooterVelocity();
        if (reversing) {
            setPower(vel < REVERSE_BELOW ? -REVERSE_POWER : 0);   // coast down first, then gently backwards
            return;
        }
        boolean steady = recordAndCheckSteady(vel);
        double error = target - vel;
        if (learning && steady && Math.abs(error) < I_ZONE) {
            integral = Math.max(-I_MAX, Math.min(I_MAX, integral + kI * error * dt));
        }
        double p = Math.max(0, Math.min(1, kF * target + kP * error + integral));   // never brakes, only coasts
        if (p > power && vel < SPINUP_BELOW * target) {
            p = Math.min(p, power + SPINUP_RAMP_PER_S * dt);   // gentle start
        }
        setPower(p);
    }

    // Keeps the last ~0.3 s of readings. Steady = now within STEADY_BAND of the reading ~0.1 s ago.
    private static final int HISTORY = 16;
    private final double[] historyTime = new double[HISTORY], historySpeed = new double[HISTORY];
    private int historyCount = 0, historyHead = 0;
    private final ElapsedTime clock = new ElapsedTime();

    private boolean recordAndCheckSteady(double vel) {
        double now = clock.seconds();
        historyTime[historyHead] = now;
        historySpeed[historyHead] = vel;
        historyHead = (historyHead + 1) % HISTORY;
        historyCount = Math.min(HISTORY, historyCount + 1);
        for (int back = 1; back < historyCount; back++) {
            int i = (historyHead - 1 - back + 2 * HISTORY) % HISTORY;
            double age = now - historyTime[i];
            if (age >= 0.08) return age <= 0.25 && Math.abs(vel - historySpeed[i]) <= STEADY_BAND;
        }
        return false;
    }

    private void setPower(double p) {
        if (hw.shooter == null || p == power || (p != 0 && Math.abs(p - power) < 0.005)) return;   // skip tiny changes
        power = p;
        hw.shooter.setPower(p);
        if (hw.secondShooter != null) hw.secondShooter.setPower(p);
    }
}
