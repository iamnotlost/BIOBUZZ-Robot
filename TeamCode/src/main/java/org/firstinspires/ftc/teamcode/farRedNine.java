package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.farRedNinePaths;

@Autonomous(name="farRedNine", group = "testing")
public class farRedNine extends LinearOpMode {
    hwManager hwManager;
    supply supply;
    shooter shooter;
    intake intake;
    Follower follower;
    farRedNinePaths paths;
    PathChain currentPath;
    int pathState = 0;

    private static final double FAR_VELOCITY = 1860;

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
        follower.setStartingPose(farRedNinePaths.startPose);

        paths = new farRedNinePaths(follower);

        telemetry.addData("init", "complete");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            shooter.shootFar();

            follower.setMaxPower(0.5);
            pathState = 1;
            currentPath = paths.Path1;
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                updateTelemetry();
            }
            follower.setMaxPower(1.0);

            sendAll();

            pathState = 2;
            currentPath = paths.Path2;
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                updateTelemetry();
            }

            intake.turnOn();
            supply.turnOnSlow();

            pathState = 3;
            currentPath = paths.Path3;
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                supply.stopOnButton(!hwManager.button.getState());
                updateTelemetry();
            }

            intake.turnOff();
            shooter.shootFar();

            pathState = 4;
            currentPath = paths.Path4;
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                updateTelemetry();
            }

            settleHeading(Math.toRadians(68.5), Math.toRadians(1.5), 400);

            sendAll();

            intake.turnOn();
            supply.turnOnSlow();

            pathState = 5;
            currentPath = paths.PreRotate5;
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                supply.stopOnButton(!hwManager.button.getState());
                updateTelemetry();
            }
            settleHeading(Math.toRadians(10), Math.toRadians(2.0), 300);

            currentPath = paths.Path5;
            follower.followPath(currentPath);
            follower.setMaxPower(0.5);
            long path5Start = System.currentTimeMillis();
            final long PATH5_TIMEOUT_MS = 4500;
            while (opModeIsActive() && follower.isBusy()
                    && System.currentTimeMillis() - path5Start < PATH5_TIMEOUT_MS) {
                follower.update();
                shooter.update();
                supply.stopOnButton(!hwManager.button.getState());
                updateTelemetry();
            }
            follower.setMaxPower(1.0);

            intake.turnOff();
            supply.turnOff();
            shooter.shootFar();

            pathState = 6;
            currentPath = paths.Path6;
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                updateTelemetry();
            }

            settleHeading(Math.toRadians(68.5), Math.toRadians(1.5), 400);

            sendAll();

            pathState = 7;
            currentPath = paths.Path7;
            follower.followPath(currentPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                shooter.update();
                updateTelemetry();
            }
            settleHeading(Math.toRadians(0), Math.toRadians(2), 400);
        }
    }

    private void settleHeading(double targetRad, double toleranceRad, long maxMs) {
        long start = System.currentTimeMillis();
        long stableStart = -1;
        final long REQUIRED_STABLE_MS = 120;

        while (opModeIsActive() && System.currentTimeMillis() - start < maxMs) {
            follower.update();
            shooter.update();

            double current = follower.getPose().getHeading();
            double error = Math.abs(normalizeAngle(current - targetRad));

            if (error < toleranceRad) {
                if (stableStart == -1) stableStart = System.currentTimeMillis();
                if (System.currentTimeMillis() - stableStart >= REQUIRED_STABLE_MS) break;
            } else {
                stableStart = -1;
            }
            updateTelemetry();
        }
    }

    private double normalizeAngle(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    public void sendAll() {
        if (shooter.currentMode != 1) {
            shooter.shootFar();
        }

        waitForShooterReady(FAR_VELOCITY, 40, 2000);

        intake.turnOn();
        supply.turnOn(520);

        activeSleep(3500);

        supply.turnOff();
        intake.turnOff();
        shooter.turnOff();
    }

    private void activeSleep(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            follower.update();
            shooter.update();
        }
    }

    private boolean waitForShooterReady(double targetVelocity, double tolerance, long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() &&
                Math.abs(hwManager.shooter.getVelocity() - targetVelocity) > tolerance) {
            follower.update();
            shooter.update();
            if (System.currentTimeMillis() - start > timeoutMs) return false;
        }
        return true;
    }

    private void updateTelemetry() {
        telemetry.addData("pathState", pathState);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.update();
    }
}
