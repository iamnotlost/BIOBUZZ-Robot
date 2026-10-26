package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// LEAVE (3 pts): drive straight off the wall and stop. Works for both alliances from any start.
@Autonomous(name="leaveAuto", group = "BIOBUZZ")
public class leaveAuto extends baseAuto {
    // Wait before moving so the alliance partner can go first
    public static double START_DELAY_S  = 0;
    public static double LEAVE_DISTANCE = 12;   // inches

    @Override
    public void runOpMode() {
        setup();
        waitForStart();
        startTimer();

        waitSeconds(START_DELAY_S);
        runPath(lineTo(LEAVE_DISTANCE, 0));

        stopAll();
    }
}
