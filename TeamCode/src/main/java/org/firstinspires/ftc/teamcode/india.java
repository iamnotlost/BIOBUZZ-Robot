package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.indiaPaths;

@Autonomous(name="colorado", group = "testing")
public class india extends LinearOpMode {
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
        follower.setStartingPose(indiaPaths.startPose);

        telemetry.addData("init", "complete");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            shooter.shootClose();

            pathState = 1;
            currentPath = indiaPaths.vRedPath1(follower);
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
            currentPath = indiaPaths.vRedPath2(follower);
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
            currentPath = indiaPaths.vRedPath3(follower);
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
            currentPath = indiaPaths.vRedPath4(follower);
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
            currentPath = indiaPaths.vRedPath5(follower);
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
            currentPath = indiaPaths.vRedPath6(follower);
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
//            shooter.shootClose();

            pathState = 7;
            currentPath = indiaPaths.vRedPath7(follower);
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

            pathState = 8;
            currentPath = indiaPaths.vRedPath8(follower);
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

            sendAll();

            pathState = 9;
            currentPath = indiaPaths.vRedPath9(follower);
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
