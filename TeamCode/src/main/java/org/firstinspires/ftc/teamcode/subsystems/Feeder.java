package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/** The feeder ("supply" in the robot configuration) carries balls up into the shooter. Positive = up. */
public class Feeder {
    public static int UP_VELOCITY = 1400;      // cross: push balls up, saved in Settings
    public static int REVERSE_VELOCITY = 800;  // circle: backwards

    private final RobotHardware hw;
    private int sentVelocity = 0;              // last velocity sent, so the motor is only told changes

    public Feeder(RobotHardware hw) {
        this.hw = hw;
    }

    /** Ticks/s, positive = up. */
    public void run(int velocity) {
        if (velocity == sentVelocity || hw.feeder == null) return;
        sentVelocity = velocity;
        hw.feeder.setVelocity(velocity);
    }

    public void stop() {
        run(0);
    }
}
