package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Speed control for the shooter wheel. Both motors (shooter + secondShooter) get the SAME power from
// one controller, so they never fight each other. The wheel speed is read from the shooter motor's encoder.
//   power = kF * target      what it takes to hold the speed
//         + kP * error       push harder when too slow, less when too fast
//         + learned extra    (kI) removes the last bit of error, e.g. on a lower battery
// Call update() once every loop.
//
// Tuned from the robot log (2026-11-07): this wheel speeds up very fast (0 -> 1300 in 0.3 s) and the
// speed reading arrives a little late, so pushing harder than this makes it overshoot after every ball
// (1380 -> 1540) and balls leave at different speeds. kP 0.001 recovers without overshooting.
// The learned extra only learns while waiting for the first ball (setLearning), never during a burst:
// learning from the dips after each ball made every next ball a bit faster. It is kept between bursts.
public class shooter {
    hwManager hwManager;

    public static double kF = 0.00037;   // power per tick/s (held 1330 with 0.49 power)
    public static double kP = 0.001;     // power per tick/s of error
    public static double kI = 0.003;     // power per (tick/s of error x second)
    private static final double I_MAX  = 0.15;   // the learned extra never goes past this
    private static final double I_ZONE = 150;    // only learn when this close (not while spinning up from 0)

    // Gentle start: while the wheel is below SPINUP_BELOW of the target, power may only go up by this much per
    // second (0 -> 100% in 0.5 s). Two motors jumping to 100% from standstill pulled so much current that the
    // battery fell to ~7 V and the Expansion Hub (shooter + feeder) cut out (2026-11-12). This halves the peak
    // current (simulated) and the wheel still reaches 1240 in ~0.6 s. After a ball (wheel still fast) there is
    // no limit, so recovery between balls is as quick as before.
    public static double SPINUP_RAMP_PER_S = 2.0;
    public static double SPINUP_BELOW      = 0.8;

    private double target = 0;
    private double integral = 0;
    private boolean learning = true;
    private double power = 0;
    private final ElapsedTime dtTimer = new ElapsedTime();

    public shooter(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    // Wanted wheel speed in ticks/s; calling it every loop is fine
    public void spinTo(int speed) {
        target = Math.max(0, speed);
    }

    // true while the wheel is just holding its speed (waiting for the first ball, pre-spin);
    // false during a burst, so the dips after each ball don't change it
    public void setLearning(boolean on) {
        learning = on;
    }

    public void turnOff() {
        target = 0;
        setPower(0);
    }

    public double power() {
        return power;
    }

    public void update() {
        double dt = Math.min(dtTimer.seconds(), 0.1);
        dtTimer.reset();
        if (hwManager.shooter == null || target <= 0) {
            setPower(0);
            return;
        }
        double vel = hwManager.shooterVelocity();
        double error = target - vel;
        if (learning && Math.abs(error) < I_ZONE) {
            integral = Math.max(-I_MAX, Math.min(I_MAX, integral + kI * error * dt));
        }
        double p = Math.max(0, Math.min(1, kF * target + kP * error + integral));   // never brakes: only coasts down
        if (p > power && vel < SPINUP_BELOW * target) {
            p = Math.min(p, power + SPINUP_RAMP_PER_S * dt);   // gentle start (see SPINUP_RAMP_PER_S)
        }
        setPower(p);
    }

    private void setPower(double p) {
        if (p == power || (p != 0 && Math.abs(p - power) < 0.005)) return;   // skip tiny changes (saves hub messages)
        power = p;
        if (hwManager.shooter != null)       hwManager.shooter.setPower(p);
        if (hwManager.secondShooter != null) hwManager.secondShooter.setPower(p);
    }
}
