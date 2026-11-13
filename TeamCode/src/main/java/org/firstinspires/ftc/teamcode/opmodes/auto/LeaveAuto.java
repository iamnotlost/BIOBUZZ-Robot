package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** LEAVE (3 pts): drive LEAVE_DISTANCE straight off the wall and stop. Works for both alliances from any start. */
@Autonomous(name = "leaveAuto", group = "BIOBUZZ")
public class LeaveAuto extends BaseAuto {
    @Override
    public void runOpMode() {
        setup();
        while (opModeInInit()) {
            follower.update();
            telemetry.addData("Auto", "LEAVE: %.0f in straight ahead", LEAVE_DISTANCE);
            showInitChecks();
            telemetry.update();
        }
        if (isStopRequested()) return;
        startTimer();

        waitSeconds(START_DELAY_S);
        step("drive straight off the wall");
        goTo("off the wall", new double[] {LEAVE_DISTANCE, 0, 0}, AUTO_SECONDS);
        finish();
    }
}
