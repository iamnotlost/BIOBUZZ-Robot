package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.redLeave;

@Autonomous(name="redLeaveAuto", group = "testing")
public class redLeaveAuto extends LinearOpMode {
    hwManager hwManager;
    Follower follower;
    PathChain currentPath;
    int pathState = 0;

    @Override
    public void runOpMode() {
        hwManager = new hwManager();
        hwManager.initHardware(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addData("init", "complete");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            pathState = 1;
            currentPath = redLeave.redLeavePath(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
        }
    }
}