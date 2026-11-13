package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequencer;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.firstinspires.ftc.teamcode.util.Settings;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/**
 * Shared AUTO code. Poses are measured from where the robot starts: +x = straight ahead, +y = to the robot's
 * left, heading 0 = the starting direction. Pedro Pathing drives; the Pinpoint is its odometry.
 *
 * Debugging: the Driver Hub shows the current STEP, where the robot is and is going, and the last EVENTS (reached
 * X 0.8 in off / GAVE UP, each ball, why shooting stopped). Every event also goes to the robot log with tag AUTO.
 * The summary stays on screen after the auto until STOP.
 *
 * Rules: everything stops SAFETY_MARGIN_S before 0:30 (G403: nothing powered in the transition).
 */
public abstract class BaseAuto extends LinearOpMode {
    public static final double AUTO_SECONDS = 30;
    public static double SAFETY_MARGIN_S = 1.0;
    public static double START_DELAY_S = 0;        // wait before moving, e.g. to let the partner go first
    public static double LEAVE_DISTANCE = 12;      // inches straight off the wall when nothing is taught
    // Never drive to a spot farther than this from the start: nothing on our half of the field is (72 x 144 in)
    public static double MAX_SPOT_IN = 165;
    // Close enough to a spot to call it reached
    public static double ARRIVED_IN = 1.0;
    public static double ARRIVED_DEG = 3;
    private static final int EVENTS_SHOWN = 7;

    protected RobotHardware hw;
    protected Drivetrain drivetrain;
    protected Intake intake;
    protected Feeder feeder;
    protected Shooter shooter;
    protected ShotSequencer shots;
    protected Follower follower;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private boolean started = false;
    private String step = "INIT";
    private String targetName = null;
    private double[] target = null;
    private final ArrayDeque<String> events = new ArrayDeque<>();

    /** Call first in runOpMode(). */
    protected void setup() {
        Settings.load();
        hw = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hw);
        intake = new Intake(hw);
        feeder = new Feeder(hw);
        shooter = new Shooter(hw);
        shots = new ShotSequencer(hw, shooter, feeder, intake);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        event(String.format("INIT %s, battery %.1f V%s", getClass().getSimpleName(), hw.batteryVoltage(),
                hw.missing.isEmpty() ? "" : ", NOT FOUND: " + hw.missingText()));
    }

    /** One loop's work: read sensors once, update the path follower and the shooter. */
    protected void tick() {
        hw.update(RobotHardware.Odometry.NONE);   // Pedro reads the Pinpoint itself
        follower.update();
        shooter.update();
    }

    /** Call right after the INIT loop ends. */
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

    // ---------------- debug screen and log ----------------

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
        return Math.toDegrees(Angles.wrap(p.getHeading()));
    }

    protected static String poseText(Pose p) {
        return String.format("%.1f in ahead, %.1f in left, turned %.0f deg", p.getX(), p.getY(), headingDeg(p));
    }

    /** Shows step, position, target and the last events. Add your own telemetry lines before calling it. */
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

    /** Checks shown on every auto's INIT screen. */
    protected void showInitChecks() {
        double volts = hw.batteryVoltage();
        telemetry.addData("Battery", "%.1f V%s", volts, volts < 12.5 ? "  !! LOW - charge or swap" : "");
        if (!hw.missing.isEmpty()) telemetry.addData("!! Not found", hw.missingText());
        telemetry.addData("Odometry", "%s  (should stay 0, 0, 0 while still)", poseText(follower.getPose()));
    }

    /** Stops everything and keeps the summary on screen until STOP (nothing is powered). */
    protected void finish() {
        stopAll();
        target = null;
        step = "DONE - press STOP";
        event("DONE, ended at " + poseText(follower.getPose()));
        while (opModeIsActive()) {
            follower.update();
            showProgress();
            sleep(100);
        }
    }

    protected void stopAll() {
        follower.breakFollowing();
        drivetrain.stop();
        shots.stop();
    }

    protected void waitSeconds(double seconds) {
        if (seconds <= 0) return;
        event(String.format("waiting %.1f s (start delay)", seconds));
        ElapsedTime t = new ElapsedTime();
        while (running() && t.seconds() < seconds) {
            tick();
            showProgress();
        }
    }

    // ---------------- driving ----------------

    protected boolean goTo(String name, double[] spot, double maxSeconds) {
        return goTo(name, spot, new ArrayList<double[]>(), maxSeconds);
    }

    /**
     * Drives to {x in, y in, heading deg} through the taught path points (in order, one smooth path), turning on
     * the way. Gives up after maxSeconds. Logs how far off it ended. Returns false if it gave up or time ran out.
     */
    protected boolean goTo(String name, double[] spot, List<double[]> via, double maxSeconds) {
        List<double[]> all = new ArrayList<>(via);
        all.add(spot);
        // Safety net: a spot this far away was taught wrong (once 103 in to the side: the robot drove across)
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

        // one straight piece per point; each turns smoothly (the short way round) to that point's heading
        Pose from = follower.getPose();
        PathBuilder builder = follower.pathBuilder();
        double fx = from.getX(), fy = from.getY(), fh = from.getHeading();
        int pieces = 0;
        for (double[] s : all) {
            double h = fh + Angles.wrap(Math.toRadians(s[2]) - fh);
            if (Math.hypot(s[0] - fx, s[1] - fy) >= 1) {   // skip points on top of the last one
                builder.addPath(new BezierLine(new Pose(fx, fy), new Pose(s[0], s[1])))
                        .setLinearHeadingInterpolation(fh, h);
                fx = s[0];
                fy = s[1];
                pieces++;
            }
            fh = h;
        }

        boolean finished;
        if (pieces > 0) {
            follower.followPath(builder.build(), false);
            finished = follow(maxSeconds, false);
        } else {
            follower.turnTo(fh);   // (almost) no distance: just turn on the spot
            finished = follow(maxSeconds, true);
        }

        Pose p = follower.getPose();
        event(String.format("%s %s in %.1f s: %.1f in off, %.0f deg off%s", finished ? "reached" : "STOPPED at",
                name, t.seconds(), Math.hypot(spot[0] - p.getX(), spot[1] - p.getY()),
                Angles.wrapDegrees(spot[2] - headingDeg(p)),
                finished ? "" : timeLeft() <= SAFETY_MARGIN_S ? " (AUTO time up)" : " (GAVE UP - blocked?)"));
        target = null;
        return finished;
    }

    // Runs the follower until the path or turn is done, the robot is close enough to the target, AUTO is about to
    // end, or maxSeconds pass. Close enough: within ARRIVED_IN / ARRIVED_DEG after a drive; for a turn on the spot
    // (aiming) within TURN_DONE_DEG of the heading. No "hold position at the end" (followPath(..., false)):
    // Pedro's hold pushed the robot back and forth around the spot.
    public static double TURN_DONE_DEG = 2;

    private boolean follow(double maxSeconds, boolean turnOnly) {
        ElapsedTime t = new ElapsedTime();
        boolean finished = false;
        while (running() && t.seconds() < maxSeconds) {
            tick();
            showProgress();
            if (!follower.isBusy() && !follower.isTurning()) { finished = true; break; }
            boolean close = turnOnly
                    ? Math.abs(Angles.wrapDegrees(target[2] - headingDeg(follower.getPose()))) < TURN_DONE_DEG
                    : closeTo(target);
            if (t.seconds() > (turnOnly ? 0.2 : 0.3) && close) { finished = true; break; }
        }
        follower.breakFollowing();
        drivetrain.stop();
        return finished;
    }

    private boolean closeTo(double[] spot) {
        Pose p = follower.getPose();
        return Math.hypot(spot[0] - p.getX(), spot[1] - p.getY()) < ARRIVED_IN
                && Math.abs(Angles.wrapDegrees(spot[2] - headingDeg(p))) < ARRIVED_DEG;
    }
}
