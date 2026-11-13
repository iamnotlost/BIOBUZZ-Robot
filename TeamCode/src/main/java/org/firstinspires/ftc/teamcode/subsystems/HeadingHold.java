package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * Heading hold (TeleOp, Share toggles it, off by default): after you let go of the turn stick it gently corrects
 * slow drift. If it would need more than GIVE_UP correction the heading reading must be wrong (the Pinpoint was
 * bumped while calibrating in INIT), so it switches itself off instead of turning the robot round and round.
 */
public class HeadingHold {
    private static final double TURN_STICK = 0.15;               // turn stick past this = driver is turning
    private static final double DELAY_MS = 400;                  // wait after letting go before holding
    private static final double KP = 0.5, MAX_POWER = 0.15;
    private static final double TOLERANCE = Math.toRadians(3);
    private static final double GIVE_UP = Math.toRadians(30);

    private boolean enabled, active, gaveUp;
    private double holdHeading;
    private final ElapsedTime delayTimer = new ElapsedTime();

    public HeadingHold(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean enabled() { return enabled; }
    public boolean gaveUp()  { return gaveUp; }

    public void toggle() {
        enabled = !enabled;
        active = false;
        gaveUp = false;
        delayTimer.reset();
    }

    public void disable() {
        enabled = false;
        active = false;
    }

    /** Forget the held heading (e.g. after resetting the heading). */
    public void reset() {
        active = false;
        delayTimer.reset();
    }

    /** Returns the turn power to use: the driver's {@code turn}, or a small correction while holding. */
    public double apply(double turn, double turnStick, double heading) {
        if (Math.abs(turnStick) > TURN_STICK) {
            reset();
            return turn;
        }
        if (enabled && !active && delayTimer.milliseconds() >= DELAY_MS) {
            holdHeading = heading;
            active = true;
        }
        if (!active) return turn;
        double error = Angles.wrap(holdHeading - heading);
        if (Math.abs(error) > GIVE_UP) {
            enabled = false;
            active = false;
            gaveUp = true;
            return turn;
        }
        return Math.abs(error) >= TOLERANCE ? Angles.clip(KP * error, MAX_POWER) : turn;
    }
}
