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
// The learned extra only learns while the wheel is STEADY (its speed changed by at most STEADY_BAND in the
// last 0.1 s) and no ball is being fed (setLearning). On 2026-11-13 it also learned while the wheel was still
// speeding up: that "windup" made the wheel hold 20-30 above the target and creep higher every burst
// (target 1190 -> 1210 -> 1230 -> 1240+). Simulated on the logged wheel: steady-only learning keeps every
// burst at 1187-1194 for a target of 1190. It is kept between bursts.
public class shooter {
    hwManager hwManager;

    public static double kF = 0.00037;   // power per tick/s (held 1330 with 0.49 power)
    public static double kP = 0.001;     // power per tick/s of error
    public static double kI = 0.003;     // power per (tick/s of error x second)
    private static final double I_MAX  = 0.15;   // the learned extra never goes past this
    private static final double I_ZONE = 150;    // only learn when this close (not while spinning up from 0)
    public static double STEADY_BAND = 20;       // "steady" = reading changed by at most this in the last 0.1 s

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
        reversing = false;
    }

    // true while no ball is being fed; false while feeding, so a ball loading the wheel doesn't change it
    public void setLearning(boolean on) {
        learning = on;
    }

    public void turnOff() {
        target = 0;
        reversing = false;
        setPower(0);
    }

    // Wheel backwards (hold circle in TeleOp: everything runs backwards so nothing stays stuck).
    // Gentle power, and only once the wheel has (nearly) stopped: throwing a fast wheel into reverse would pull a
    // huge current from the battery (that is what made the hubs cut out before).
    public static double REVERSE_POWER = 0.3;
    public static double REVERSE_BELOW = 300;   // ticks/s: wheel slower than this may be reversed
    private boolean reversing = false;

    public void reverse() {
        target = 0;
        reversing = true;
    }

    public double power() {
        return power;
    }

    public void update() {
        double dt = Math.min(dtTimer.seconds(), 0.1);
        dtTimer.reset();
        if (reversing && hwManager.shooter != null) {
            // coast until the wheel is slow, then run it gently backwards
            setPower(hwManager.shooterVelocity() < REVERSE_BELOW ? -REVERSE_POWER : 0);
            return;
        }
        if (hwManager.shooter == null || target <= 0) {
            setPower(0);
            return;
        }
        double vel = hwManager.shooterVelocity();
        double error = target - vel;
        boolean steady = steady(vel);
        if (learning && steady && Math.abs(error) < I_ZONE) {
            integral = Math.max(-I_MAX, Math.min(I_MAX, integral + kI * error * dt));
        }
        double p = Math.max(0, Math.min(1, kF * target + kP * error + integral));   // never brakes: only coasts down
        if (p > power && vel < SPINUP_BELOW * target) {
            p = Math.min(p, power + SPINUP_RAMP_PER_S * dt);   // gentle start (see SPINUP_RAMP_PER_S)
        }
        setPower(p);
    }

    // Remembers the readings of the last ~0.3 s; steady = the reading now is within STEADY_BAND of the one
    // from about 0.1 s ago (so the wheel is not speeding up or slowing down)
    private static final int HIST = 16;
    private final double[] histT = new double[HIST], histV = new double[HIST];
    private int histN = 0, histHead = 0;
    private final ElapsedTime clock = new ElapsedTime();

    private boolean steady(double vel) {
        double now = clock.seconds();
        histT[histHead] = now;
        histV[histHead] = vel;
        histHead = (histHead + 1) % HIST;
        histN = Math.min(HIST, histN + 1);
        for (int back = 1; back < histN; back++) {
            int i = (histHead - 1 - back + 2 * HIST) % HIST;
            double age = now - histT[i];
            if (age >= 0.08) return age <= 0.25 && Math.abs(vel - histV[i]) <= STEADY_BAND;
        }
        return false;
    }

    private void setPower(double p) {
        if (p == power || (p != 0 && Math.abs(p - power) < 0.005)) return;   // skip tiny changes (saves hub messages)
        power = p;
        if (hwManager.shooter != null)       hwManager.shooter.setPower(p);
        if (hwManager.secondShooter != null) hwManager.secondShooter.setPower(p);
    }
}
