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
        settings.load(); // park distances and start delay from settingsMenu
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

    // Drives to a spot {x in, y in, heading deg} from autoSpots, turning on the way. Gives up after maxSeconds.
    protected void goTo(double[] spot, double maxSeconds) {
        Pose from = follower.getPose();
        // turn the short way round (e.g. 30 deg right, not 330 deg left)
        double turn = Math.toRadians(spot[2]) - from.getHeading();
        turn = Math.atan2(Math.sin(turn), Math.cos(turn));
        double heading = from.getHeading() + turn;
        if (Math.hypot(spot[0] - from.getX(), spot[1] - from.getY()) < 1) {
            // (almost) no distance to drive: just turn on the spot
            follower.turnTo(heading);
            ElapsedTime t = new ElapsedTime();
            while (running() && (follower.isBusy() || follower.isTurning()) && t.seconds() < maxSeconds) {
                follower.update();
                shooter.update();
                showProgress();
                double left = heading - follower.getPose().getHeading();
                if (t.seconds() > 0.2 && Math.abs(Math.atan2(Math.sin(left), Math.cos(left))) < Math.toRadians(2)) break;
            }
            follower.breakFollowing();
            return;
        }
        runPath(follower.pathBuilder()
                .addPath(new BezierLine(new Pose(from.getX(), from.getY()), new Pose(spot[0], spot[1])))
                .setLinearHeadingInterpolation(from.getHeading(), heading)
                .build(), maxSeconds);
    }

    protected void runPath(PathChain path) {
        runPath(path, AUTO_SECONDS);
    }

    // Follows a path until it is done, AUTO is about to end, or maxSeconds have passed (e.g. blocked)
    protected void runPath(PathChain path, double maxSeconds) {
        follower.followPath(path);
        ElapsedTime t = new ElapsedTime();
        while (running() && follower.isBusy() && t.seconds() < maxSeconds) {
            follower.update();
            shooter.update();
            showProgress();
        }
        follower.breakFollowing();
    }

    protected void showProgress() {
        Pose p = follower.getPose();
        telemetry.addData("Time left", "%.1f", timeLeft());
        telemetry.addData("Robot at", "%.0f in ahead, %.0f in left, turned %.0f deg",
                p.getX(), p.getY(), Math.toDegrees(Math.atan2(Math.sin(p.getHeading()), Math.cos(p.getHeading()))));
        telemetry.update();
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
