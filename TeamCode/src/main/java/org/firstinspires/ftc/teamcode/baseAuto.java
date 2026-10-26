package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Shared AUTO helpers. Poses are relative to where the robot starts:
// +x = straight ahead (away from the wall), +y = to the robot's left, heading 0 = starting direction.
public abstract class baseAuto extends LinearOpMode {
    public static final double AUTO_SECONDS = 30;
    // Stop moving this long before AUTO ends so nothing is still powered at 0:00 (G403)
    public static double SAFETY_MARGIN_S = 1.0;

    hwManager hwManager;
    drivetrain drivetrain;
    supply supply;
    shooter shooter;
    intake intake;
    Follower follower;
    final ElapsedTime autoTimer = new ElapsedTime();

    protected void setup() {
        hwManager = new hwManager();
        hwManager.initHardware(hardwareMap);
        drivetrain = new drivetrain(hwManager);
        supply = new supply(hwManager);
        shooter = new shooter(hwManager);
        intake = new intake(hwManager);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addData("init", "complete");
        telemetry.addData("Missing", hwManager.missingText());
        telemetry.update();
    }

    // Call right after waitForStart()
    protected void startTimer() {
        autoTimer.reset();
    }

    protected double timeLeft() {
        return AUTO_SECONDS - autoTimer.seconds();
    }

    protected boolean running() {
        return opModeIsActive() && timeLeft() > SAFETY_MARGIN_S;
    }

    // Straight line to (x, y) inches from the start, keeping the starting heading
    protected PathChain lineTo(double x, double y) {
        Pose from = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(new Pose(from.getX(), from.getY()), new Pose(x, y)))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    // Follows a path until it is done or AUTO is about to end
    protected void runPath(PathChain path) {
        follower.followPath(path);
        while (running() && follower.isBusy()) {
            follower.update();
            shooter.update();
            telemetry.addData("Time left", "%.1f", timeLeft());
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }
        follower.breakFollowing();
    }

    protected void waitSeconds(double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (running() && t.seconds() < seconds) {
            follower.update();
            shooter.update();
        }
    }

    protected void stopAll() {
        follower.breakFollowing();
        drivetrain.stop();
        shooter.turnOff();
        supply.turnOff();
        intake.stop();
    }
}
