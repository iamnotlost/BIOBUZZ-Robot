package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.closeRedPaths;

@Autonomous(name="closeRed", group = "testing")
public class closeRed extends LinearOpMode {
    hwManager hwManager;
    supply supply;
    shooter shooter;
    intake intake;
    Follower follower;
    PathChain currentPath;
    int pathState = 0;

    @Override
    public void runOpMode() {
        hwManager = new hwManager();
        hwManager.initHardware(hardwareMap);
        sleep(300);

        hwManager.pinpoint.update();

        supply = new supply(hwManager);
        shooter = new shooter(hwManager);
        intake = new intake(hwManager);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(closeRedPaths.startPose);

        telemetry.addData("init", "complete");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            shooter.shootClose();

            pathState = 1;
            currentPath = closeRedPaths.vRedPath1(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            sendAll();

            pathState = 2;
            currentPath = closeRedPaths.vRedPath2(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            intake.turnOn();
            supply.turnOnSlow();

            pathState = 3;
            currentPath = closeRedPaths.vRedPath3(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                supply.stopOnButton(!hwManager.button.getState());
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            intake.turnOff();

            pathState = 4;
            currentPath = closeRedPaths.vRedPath4(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            shooter.shootClose();

            pathState = 5;
            currentPath = closeRedPaths.vRedPath5(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            sendAll();

            intake.turnOn();
            supply.turnOnSlow();

            pathState = 6;
            currentPath = closeRedPaths.vRedPath6(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                supply.stopOnButton(!hwManager.button.getState());
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            intake.turnOff();
            shooter.shootClose();

            pathState = 7;
            currentPath = closeRedPaths.vRedPath7(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            sendAll();

            intake.turnOn();
            supply.turnOnSlow();

            pathState = 8;
            currentPath = closeRedPaths.vRedPath8(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                supply.stopOnButton(!hwManager.button.getState());
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            intake.turnOff();
            shooter.shootClose();

            pathState = 9;
            currentPath = closeRedPaths.vRedPath9(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }

            sendAll();

            pathState = 10;
            currentPath = closeRedPaths.vRedPath10(follower);
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                telemetry.addData("pathState", pathState);
                telemetry.addData("pose", follower.getPose());
                telemetry.addData("isBusy", follower.isBusy());
                telemetry.update();
            }
        }
    }

    public void sendAll() {
        if (shooter.currentMode != 2) {
            shooter.shootClose();
            waitAndUpdate(700);
        }

        waitAndUpdate(100);
        supply.turnOn();
        waitAndUpdate(150);
        intake.turnOn();

        waitAndUpdate(1800);

        shooter.turnOff();
        supply.turnOff();
        intake.turnOff();
    }

    private void waitAndUpdate(long ms) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < ms) {
            shooter.update();
        }
    }
}
