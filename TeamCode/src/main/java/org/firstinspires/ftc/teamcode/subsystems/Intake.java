package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Front roller. Must run slowly: above ~300 ticks/s the rubber bands stretch and fly off.
 * {@link #load()} pulls balls in and, if the roller is stuck, briefly runs it backwards to free it.
 */
public class Intake {
    public static int IN_VELOCITY = 300;          // ticks/s, gamepad 2 D-pad up/down, saved in Settings
    public static final int SPEED_STEP = 100, MIN_SPEED = 100, MAX_SPEED = 2500;

    // Unjam: on but turning slower than JAM_FRACTION of the set speed for JAM_SECONDS -> backwards UNJAM_SECONDS
    public static double JAM_SECONDS = 0.4, UNJAM_SECONDS = 0.3, JAM_FRACTION = 0.3;

    private final RobotHardware hw;
    private int sentVelocity = 0;                  // last velocity sent, so the motor is only told changes
    private boolean unjamming = false;
    private final ElapsedTime jamTimer = new ElapsedTime(), unjamTimer = new ElapsedTime();

    public Intake(RobotHardware hw) {
        this.hw = hw;
    }

    public static void adjustSpeed(int steps) {
        IN_VELOCITY = Math.max(MIN_SPEED, Math.min(MAX_SPEED, IN_VELOCITY + steps * SPEED_STEP));
    }

    /** Pull balls in (no jam check), e.g. while shooting. */
    public void in() {
        run(IN_VELOCITY);
        resetJam();
    }

    /** Push balls out. */
    public void out() {
        run(-IN_VELOCITY);
        resetJam();
    }

    public void stop() {
        run(0);
        resetJam();
    }

    /** Pull balls in, freeing the roller if it gets stuck. Call every loop while loading. */
    public void load() {
        if (unjamming) {
            run(-IN_VELOCITY);
            if (unjamTimer.seconds() >= UNJAM_SECONDS) {
                unjamming = false;
                jamTimer.reset();
            }
            return;
        }
        run(IN_VELOCITY);
        boolean slow = Math.abs(hw.intakeVelocity()) < IN_VELOCITY * JAM_FRACTION;
        if (!slow) {
            jamTimer.reset();
        } else if (jamTimer.seconds() >= JAM_SECONDS) {
            unjamming = true;
            unjamTimer.reset();
        }
    }

    public boolean unjamming() {
        return unjamming;
    }

    private void resetJam() {
        unjamming = false;
        jamTimer.reset();
    }

    private void run(int velocity) {
        if (velocity == sentVelocity || hw.intake == null) return;
        sentVelocity = velocity;
        hw.intake.setVelocity(velocity);
    }
}
