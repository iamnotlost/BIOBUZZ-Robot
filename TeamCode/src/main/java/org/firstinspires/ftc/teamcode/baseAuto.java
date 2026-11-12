package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayDeque;

// Shared AUTO helpers. Poses are relative to where the robot starts:
// +x = straight ahead (away from the wall), +y = to the robot's left, heading 0 = starting direction.
//
// Debugging: the Driver Hub shows the current STEP, where the robot is and where it is going, and the last
// few EVENTS (arrived how far off, gave up, each ball, why shooting stopped). Every event also goes to the
// robot log with tag AUTO, so a run can be read back afterwards (adb pull /sdcard/robotControllerLog.txt).
// After the auto finishes the screen keeps the summary until STOP.
public abstract class baseAuto extends LinearOpMode {
    public static final double AUTO_SECONDS = 30;
    // Stop moving this long before AUTO ends so nothing is still powered at 0:00 (G403)
    public static double SAFETY_MARGIN_S = 1.0;
    // Never drive to a taught spot farther than this from the start (inches): nothing on our half of the field is
    // farther (72 x 144 in), so it must have been taught wrong. (100 was too small: a real LOADING ZONE spot was 115.)
    public static double MAX_SPOT_IN = 165;
    private static final int EVENTS_SHOWN = 7;

    hwManager hwManager;
    drivetrain drivetrain;
    supply supply;
    shooter shooter;
    intake intake;
    Follower follower;
    final ElapsedTime autoTimer = new ElapsedTime();
    private boolean started = false;

    private String step = "INIT";
    private String targetName = null;
    private double[] target = null;
    private final ArrayDeque<String> events = new ArrayDeque<>();

    protected void setup() {
        settings.load(); // spots, park distances and start delay
        hwManager = new hwManager();
        hwManager.initHardware(hardwareMap);
        drivetrain = new drivetrain(hwManager);
        supply = new supply(hwManager);
        shooter = new shooter(hwManager);
        intake = new intake(hwManager);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        event("INIT " + getClass().getSimpleName() + ", battery " + String.format("%.1f V", hwManager.batteryVoltage())
                + (hwManager.missing.isEmpty() ? "" : ", NOT FOUND: " + hwManager.missingText()));
    }

    // Call right after waitForStart()
    protected void startTimer() {
        autoTimer.reset();
        started = true;
        event("START");
    }

    protected double timeLeft() {
        return AUTO_SECONDS - autoTimer.seconds();
    }

    protected boolean running() {
        return opModeIsActive() && timeLeft() > SAFETY_MARGIN_S;
    }

    // ---------- debug ----------

    protected void step(String name) {
        step = name;
        event("STEP " + name);
    }

    protected void event(String message) {
        String line = (started ? String.format("%4.1fs ", autoTimer.seconds()) : "init  ") + message;
        RobotLog.ii("AUTO", line);
        events.addFirst(line);
        while (events.size() > EVENTS_SHOWN) events.removeLast();
    }

    protected static double headingDeg(Pose p) {
        return Math.toDegrees(Math.atan2(Math.sin(p.getHeading()), Math.cos(p.getHeading())));
    }

    protected static String poseText(Pose p) {
        return String.format("%.1f in ahead, %.1f in left, turned %.0f deg", p.getX(), p.getY(), headingDeg(p));
    }

    // Shows step, position, target and the last events. Add your own telemetry lines before calling it.
    protected void showProgress() {
        Pose p = follower.getPose();
        telemetry.addData("Step", step);
        if (started) telemetry.addData("Time left", "%.1f s", timeLeft());
        telemetry.addData("Robot at", poseText(p));
        if (target != null) {
            telemetry.addData("Going to", "%s: %.0f, %.0f, %.0f deg  (%.1f in to go)", targetName,
                    target[0], target[1], target[2], Math.hypot(target[0] - p.getX(), target[1] - p.getY()));
        }
        telemetry.addLine("--- what happened (newest first) ---");
        for (String e : events) telemetry.addLine(e);
        telemetry.update();
    }

    // After the auto is done: keep the summary on screen until STOP (nothing is powered)
    protected void finish() {
        stopAll();
        target = null;
        step = "DONE - press STOP";
        event(String.format("DONE, ended at %s", poseText(follower.getPose())));
        while (opModeIsActive()) {
            follower.update();
            showProgress();
            sleep(100);
        }
    }

    // ---------- driving ----------

    // Straight line to (x, y) inches from the start, keeping the starting heading
    protected PathChain lineTo(double x, double y) {
        Pose from = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(new Pose(from.getX(), from.getY()), new Pose(x, y)))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    // Drives to a spot {x in, y in, heading deg}, turning on the way. Gives up after maxSeconds.
    // Logs how far off it ended up. Returns false if it gave up (blocked?) or AUTO ran out of time.
    protected boolean goTo(String name, double[] spot, double maxSeconds) {
        return goTo(name, spot, new java.util.ArrayList<double[]>(), maxSeconds);
    }

    // Same, but drives THROUGH the taught path points first (in order, one smooth path, no stops), so the robot
    // doesn't take a straight line through things.
    protected boolean goTo(String name, double[] spot, java.util.List<double[]> via, double maxSeconds) {
        // Safety net: a spot this far from the start was taught wrong (it once was 103 in to the side and the
        // robot drove across the field). Don't drive there at all.
        java.util.List<double[]> all = new java.util.ArrayList<>(via);
        all.add(spot);
        for (double[] s : all) {
            double far = Math.hypot(s[0], s[1]);
            if (far > MAX_SPOT_IN) {
                event(String.format("%s%s is %.0f in from START - too far, NOT driving there (teach it again)",
                        name, s == spot ? "" : " path point", far));
                return false;
            }
        }
        targetName = name;
        target = spot;
        event(String.format("drive to %s (%.0f, %.0f, %.0f deg)%s", name, spot[0], spot[1], spot[2],
                via.isEmpty() ? "" : " via " + via.size() + " path point" + (via.size() > 1 ? "s" : "")));
        ElapsedTime t = new ElapsedTime();
        Pose from = follower.getPose();
        // one straight piece per path point, then to the spot; each piece turns smoothly to that point's heading
        // (always the short way round, e.g. 30 deg right, not 330 deg left)
        com.pedropathing.paths.PathBuilder builder = follower.pathBuilder();
        double fx = from.getX(), fy = from.getY(), fh = from.getHeading();
        int pieces = 0;
        for (double[] s : all) {
            double h = fh + Math.atan2(Math.sin(Math.toRadians(s[2]) - fh), Math.cos(Math.toRadians(s[2]) - fh));
            if (Math.hypot(s[0] - fx, s[1] - fy) < 1) { fh = h; continue; }   // too close to the last point
            builder.addPath(new BezierLine(new Pose(fx, fy), new Pose(s[0], s[1]))).setLinearHeadingInterpolation(fh, h);
            fx = s[0];
            fy = s[1];
            fh = h;
            pieces++;
        }
        double heading = fh;
        boolean finished;
        if (pieces == 0) {
            // (almost) no distance to drive: just turn on the spot
            follower.turnTo(heading);
            finished = false;
            while (running() && (follower.isBusy() || follower.isTurning()) && t.seconds() < maxSeconds) {
                follower.update();
                shooter.update();
                showProgress();
                double left = heading - follower.getPose().getHeading();
                if (t.seconds() > 0.2 && Math.abs(Math.atan2(Math.sin(left), Math.cos(left))) < Math.toRadians(2)) {
                    finished = true;
                    break;
                }
            }
            if (!follower.isBusy() && !follower.isTurning()) finished = true;
            follower.breakFollowing();
        } else {
            finished = runPath(builder.build(), maxSeconds);
        }
        Pose p = follower.getPose();
        double off = Math.hypot(spot[0] - p.getX(), spot[1] - p.getY());
        double turnOff = spot[2] - headingDeg(p);
        turnOff = Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(turnOff)), Math.cos(Math.toRadians(turnOff))));
        event(String.format("%s %s in %.1f s: %.1f in off, %.0f deg off%s", finished ? "reached" : "STOPPED at",
                name, t.seconds(), off, turnOff,
                finished ? "" : (timeLeft() <= SAFETY_MARGIN_S ? " (AUTO time up)" : " (GAVE UP - blocked?)")));
        target = null;
        return finished;
    }

    protected boolean runPath(PathChain path) {
        return runPath(path, AUTO_SECONDS);
    }

    // Close enough to the spot to call it reached
    public static double ARRIVED_IN  = 1.0;
    public static double ARRIVED_DEG = 3;

    // Follows a path until it is done, AUTO is about to end, or maxSeconds have passed (e.g. blocked).
    // Returns true if the path was finished.
    // No "hold position at the end": Pedro's hold kept pushing the robot forward and backward around the
    // spot (2026-11-12). The robot stops as soon as it is within ARRIVED_IN / ARRIVED_DEG of the spot.
    protected boolean runPath(PathChain path, double maxSeconds) {
        follower.followPath(path, false);
        ElapsedTime t = new ElapsedTime();
        boolean finished = false;
        while (running() && t.seconds() < maxSeconds) {
            follower.update();
            shooter.update();
            showProgress();
            if (!follower.isBusy()) { finished = true; break; }
            if (target != null && t.seconds() > 0.3 && closeTo(target)) { finished = true; break; }
        }
        follower.breakFollowing();
        drivetrain.stop();
        return finished;
    }

    private boolean closeTo(double[] spot) {
        Pose p = follower.getPose();
        double turnOff = Math.toRadians(spot[2]) - p.getHeading();
        turnOff = Math.abs(Math.toDegrees(Math.atan2(Math.sin(turnOff), Math.cos(turnOff))));
        return Math.hypot(spot[0] - p.getX(), spot[1] - p.getY()) < ARRIVED_IN && turnOff < ARRIVED_DEG;
    }

    protected void waitSeconds(double seconds) {
        if (seconds <= 0) return;
        event(String.format("waiting %.1f s (start delay)", seconds));
        ElapsedTime t = new ElapsedTime();
        while (running() && t.seconds() < seconds) {
            follower.update();
            shooter.update();
            showProgress();
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
