package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequencer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.firstinspires.ftc.teamcode.util.AutoSpots;
import org.firstinspires.ftc.teamcode.util.Settings;

import java.util.ArrayList;
import java.util.List;

/**
 * Teaches an auto its route by driving it by hand (INIT anywhere, keep still, then START):
 * <pre>
 *   SQUARE        1. START here     (robot on its start spot, touching the wall)
 *   L1            path point here   (the auto drives THROUGH it; up to 5 per route, D-pad down = undo)
 *   CROSS         2. SHOOT here     (shooting setup only; at the start = shoot without driving)
 *   CIRCLE        3. LOADING ZONE   (partly in it, NOT touching the wall)
 *   hold TRIANGLE test-shoot from here (shooting setup only)
 *   hold R1       load balls
 *   sticks        drive at half speed, a trigger = quarter speed
 * </pre>
 * Spots and points are measured from where SQUARE was pressed (not from the INIT spot), with the same odometry the
 * autos use. Path points before CROSS belong to start -> shoot, after it to shoot -> LOADING ZONE.
 * Park version (SetupParkRed / SetupParkBlue): SQUARE on the corner start, CIRCLE in the LOADING ZONE.
 */
public abstract class SetupAuto extends OpMode {
    public static double DRIVE_SCALE = 0.5;
    public static double SLOW_SCALE = 0.25;
    public static double FAR_IN = 165;   // farther than this from START can't be on our half of the field

    protected abstract Alliance alliance();

    /** true = teach the safe park auto (parkRed / parkBlue) instead of the shooting auto. */
    protected boolean parkOnly() {
        return false;
    }

    private RobotHardware hw;
    private Intake intake;
    private Shooter shooter;
    private ShotSequencer shots;
    private Follower follower;

    private Pose startPose = null;                          // where SQUARE was pressed
    private final List<double[]> routePoints = new ArrayList<>();   // path points of the route being driven
    private boolean shootSaved, endSaved;
    private String message = "";
    private final OdometryCheck odometryCheck = new OdometryCheck();

    @Override
    public void init() {
        Settings.load();
        hw = new RobotHardware(hardwareMap);
        intake = new Intake(hw);
        shooter = new Shooter(hw);
        shots = new ShotSequencer(hw, shooter, new Feeder(hw), intake);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void init_loop() {
        follower.update();
        if (!parkOnly()) {   // D-pad left/right picks which plan to teach
            if (gamepad1.dpadLeftWasPressed()) AutoSpots.plan = 0;
            if (gamepad1.dpadRightWasPressed()) AutoSpots.plan = 1;
        }
        telemetry.addLine(title() + (parkOnly() ? "" : "   (D-pad left = plan A, right = plan B)"));
        telemetry.addLine("Keep the robot STILL during INIT (odometry calibrating), then press START (play).");
        telemetry.addLine("Then drive to the " + (parkOnly() ? "CORNER " : "") + "START spot and press SQUARE.");
        showSaved();
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);   // true = brake when you let go
    }

    @Override
    public void loop() {
        hw.update(RobotHardware.Odometry.NONE);   // Pedro reads the Pinpoint itself
        follower.update();
        Pose p = follower.getPose();
        drive();
        handleTeachButtons(p);
        handleBalls();
        shooter.update();
        showScreen(p);
    }

    @Override
    public void stop() {
        shots.stop();
        follower.breakFollowing();
    }

    // Same stick feel as TeleOp
    private void drive() {
        double scale = Math.max(gamepad1.left_trigger, gamepad1.right_trigger) > 0.5 ? SLOW_SCALE : DRIVE_SCALE;
        double fwd = -gamepad1.left_stick_y, left = -gamepad1.left_stick_x;
        double push = Math.hypot(fwd, left);
        double power = Drivetrain.shape(push, scale, Drivetrain.MIN_DRIVE_POWER);
        double fwdCmd = push > 0 ? fwd / push * power : 0;
        double leftCmd = push > 0 ? left / push * power : 0;
        double turnCmd = -Drivetrain.shape(gamepad1.right_stick_x, scale, Drivetrain.MIN_TURN_POWER);
        follower.setTeleOpDrive(fwdCmd, leftCmd, turnCmd, true);
        odometryCheck.update(follower.getPose(), fwdCmd, leftCmd, turnCmd);
        if (odometryCheck.justWentBad()) {
            RobotLog.ww("SETUP", "ODOMETRY WRONG WAY: %s", odometryCheck.text());
            gamepad1.rumble(1000);
        }
    }

    private void handleTeachButtons(Pose p) {
        // read every button once per loop (the "was pressed" flags reset when read)
        boolean square = gamepad1.squareWasPressed(), cross = gamepad1.crossWasPressed();
        boolean circle = gamepad1.circleWasPressed(), point = gamepad1.leftBumperWasPressed();
        boolean undo = gamepad1.dpadDownWasPressed();

        if (square) {
            startPose = p;
            shootSaved = endSaved = false;
            routePoints.clear();
            gamepad1.rumble(200);
            RobotLog.ii("SETUP", "%s %s START set (odometry check: %s, battery %.1f V)", alliance(),
                    parkOnly() ? "PARK" : "PLAN " + AutoSpots.planName(AutoSpots.plan), odometryCheck.text(), hw.batteryVoltage());
            message = parkOnly() ? "START set. Now drive into the LOADING ZONE (L1 = path points) and press CIRCLE."
                    : "START set. Drive to where it should shoot (L1 = path points) and press CROSS.";
        }
        if (cross && parkOnly()) {
            message = "CROSS does nothing here: SQUARE = START, CIRCLE = LOADING ZONE";
        }
        if ((point || undo || (cross && !parkOnly()) || circle) && startPose == null) {
            message = "!! First put the robot on its START spot and press SQUARE.";
            gamepad1.rumble(800);
            return;
        }

        if (point) {
            if (routePoints.size() >= AutoSpots.MAX_POINTS) {
                message = "!! " + AutoSpots.MAX_POINTS + " path points is the most per route";
                gamepad1.rumble(800);
            } else {
                routePoints.add(rounded(relative(p)));
                message = "Path point " + routePoints.size() + " added: " + AutoSpots.text(routePoints.get(routePoints.size() - 1));
                gamepad1.rumble(150);
            }
        }
        if (undo && !routePoints.isEmpty()) {
            routePoints.remove(routePoints.size() - 1);
            message = "Last path point removed (" + routePoints.size() + " left)";
        }

        int plan = AutoSpots.plan;
        String planText = "PLAN " + AutoSpots.planName(plan);
        if (cross && !parkOnly()) {
            message = save(AutoSpots.shoot(alliance(), plan), AutoSpots.route(alliance(), plan, true), p, planText + " SHOOT");
            if (!AutoSpots.isSet(AutoSpots.shoot(alliance(), plan))) message += " (= START spot: shoots without driving)";
            shootSaved = true;
        }
        if (circle) {
            message = parkOnly()
                    ? save(AutoSpots.park(alliance()), AutoSpots.parkRoute(alliance()), p, "SAFE PARK")
                    : save(AutoSpots.end(alliance(), plan), AutoSpots.route(alliance(), plan, false), p, planText + " LOADING ZONE");
            endSaved = true;
        }
    }

    // hold triangle = test-shoot (shooting setup only), hold R1 = load
    private void handleBalls() {
        if (gamepad1.triangle && !parkOnly()) {
            if (!shots.active()) shots.start();
            shots.update();
            return;
        }
        if (shots.active()) shots.stop();
        if (gamepad1.right_bumper) intake.load();
        else intake.stop();
    }

    /** Saves a spot and the path points driven to it, measured from START. Returns the message to show. */
    private String save(double[] spot, String route, Pose p, String name) {
        double[] r = rounded(relative(p));
        System.arraycopy(r, 0, spot, 0, 3);
        AutoSpots.setPoints(route, routePoints);
        String via = routePoints.isEmpty() ? " (straight line)"
                : " (via " + routePoints.size() + " path point" + (routePoints.size() > 1 ? "s)" : ")");
        routePoints.clear();
        RobotLog.ii("SETUP", "%s %s saved: %.0f ahead, %.0f left, %.0f deg from START%s (odometry check: %s, battery %.1f V)",
                alliance(), name, spot[0], spot[1], spot[2], via, odometryCheck.text(), hw.batteryVoltage());

        boolean ok = Settings.save();
        gamepad1.rumble(ok ? 200 : 800);
        String msg = ok ? name + " SAVED: " + AutoSpots.text(spot) + " from START" + via : name + " could NOT be saved";
        double far = Math.hypot(spot[0], spot[1]);
        if (far > FAR_IN) msg += String.format("  !! that is %.0f ft from START - was SQUARE pressed on the start spot?", far / 12);
        if (odometryCheck.bad()) msg += "  !! odometry counts the wrong way - this spot is wrong";
        return msg;
    }

    /** Where the robot is now, measured from START: {inches ahead, inches left, degrees turned left}. */
    private double[] relative(Pose p) {
        double dx = p.getX() - startPose.getX(), dy = p.getY() - startPose.getY(), h0 = startPose.getHeading();
        return new double[] {
                dx * Math.cos(h0) + dy * Math.sin(h0),
                -dx * Math.sin(h0) + dy * Math.cos(h0),
                Math.toDegrees(Angles.wrap(p.getHeading() - h0))};
    }

    private static double[] rounded(double[] v) {
        return new double[] {Math.round(v[0]), Math.round(v[1]), Math.round(v[2])};
    }

    // ---------------- screen ----------------

    private String title() {
        return parkOnly() ? "SETUP SAFE PARK " + alliance()
                : "SETUP SHOOT AUTO " + alliance() + "   PLAN " + AutoSpots.planName(AutoSpots.plan);
    }

    private void showScreen(Pose p) {
        telemetry.addLine(title());
        telemetry.addLine(check(startPose != null) + "SQUARE = 1. START here (robot on its start spot, touching the wall)");
        telemetry.addLine("     L1 = path point here (to drive around things), D-pad down = undo   points on this route: " + routePoints.size());
        if (!parkOnly()) telemetry.addLine(check(shootSaved) + "CROSS = 2. SHOOT here   (hold TRIANGLE to test the aim, R1 = load)");
        telemetry.addLine(check(endSaved) + "CIRCLE = " + (parkOnly() ? "2" : "3") + ". LOADING ZONE here (partly in it, NOT touching the wall)");
        if (startPose != null) {
            double[] r = relative(p);
            telemetry.addData("From START", "%.0f in ahead, %.0f in left, turned %.0f deg", r[0], r[1], r[2]);
        } else {
            telemetry.addData("From START", "START not set yet - press SQUARE on the start spot");
        }
        telemetry.addData("Odometry check", odometryCheck.text());
        double volts = hw.batteryVoltage();
        if (volts < 12.5) telemetry.addData("!! Battery", "%.1f V - LOW: motors may cut out, charge or swap", volts);
        if (shots.active()) telemetry.addData("Shooter", shots.status());
        if (!message.isEmpty()) telemetry.addLine(message);
        showSaved();
        telemetry.addLine("Rules: stay on your side (G402), don't touch the HIVE (G417).");
        telemetry.update();
    }

    private static String check(boolean done) {
        return done ? "[OK] " : "[  ] ";
    }

    private void showSaved() {
        if (parkOnly()) {
            telemetry.addData("Saved safe park spot", AutoSpots.text(AutoSpots.park(alliance())));
            return;
        }
        int plan = AutoSpots.plan;
        telemetry.addData("Saved plan " + AutoSpots.planName(plan) + " shoot", AutoSpots.shootText(alliance(), plan));
        telemetry.addData("Saved plan " + AutoSpots.planName(plan) + " loading zone", AutoSpots.text(AutoSpots.end(alliance(), plan)));
    }
}
