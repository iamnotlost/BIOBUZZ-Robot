package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Teach the shooting auto where to go. INIT anywhere, press START, then drive the robot by hand:
//   Square (□)                 1. START is here   (robot on its start spot, touching the wall)
//   Cross (X)                  2. SHOOT here      (press it right at the start to shoot without driving)
//   Circle (O)                 3. LOADING ZONE    (end spot: partly in the LOADING ZONE, NOT touching the wall)
//   Left stick / right stick   drive / turn (half speed; hold a trigger for quarter speed)
//   L1                         path point here: the auto drives THROUGH it on the way (around the HIVE, the
//                              partner robot...). Up to 5 per route, in order. D-pad down = undo the last one.
//   Hold triangle              shoot from here, to check the aim (same burst as the auto)
//   Hold R1                    intake, to load balls again for another test
// SHOOT and LOADING ZONE are saved measured from where you pressed □, so it does not matter where the robot was
// at INIT. (Before □ existed the start was the INIT spot, and spots taught after INIT somewhere else ended up
// 100 inches off.) Positions come from the same odometry the auto uses.
// Park-only version (setupParkRed / setupParkBlue): □ on the CORNER start spot, O in the LOADING ZONE saves the
// SAFE PARK spot used by parkRed / parkBlue.
public abstract class baseSetupAuto extends OpMode {
    protected abstract Alliance alliance();

    // true = teach the safe park auto (parkRed / parkBlue) instead of the shooting auto
    protected boolean parkOnly() {
        return false;
    }

    public static double DRIVE_SCALE = 0.5;
    public static double SLOW_SCALE  = 0.25;
    // A spot this far from START can't be on our half of the field (START pressed in the wrong place?)
    public static double FAR_IN = 165;

    hwManager hwManager;
    shooter shooter;
    supply supply;
    intake intake;
    burst burst;
    Follower follower;

    private boolean prevCross, prevCircle, prevTriangle, prevSquare, prevLeft, prevRight, prevL1, prevDown;
    // path points taught since the last START / SHOOT save; they belong to the route being driven now
    private final java.util.List<double[]> legPoints = new java.util.ArrayList<>();
    private boolean loading = false;
    private String message = "";
    private Pose startPose = null;          // where □ was pressed
    private boolean shootSaved = false, endSaved = false;

    @Override
    public void init() {
        settings.load();
        hwManager = new hwManager();
        hwManager.initHardware(hardwareMap);
        shooter = new shooter(hwManager);
        supply = new supply(hwManager);
        intake = new intake(hwManager);
        burst = new burst(hwManager, shooter, supply, intake);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void init_loop() {
        follower.update();
        // shooting auto: D-pad left/right picks which plan (A or B, two different start spots) to teach
        if (!parkOnly()) {
            if (gamepad1.dpad_left && !prevLeft)   autoSpots.plan = 0;
            if (gamepad1.dpad_right && !prevRight) autoSpots.plan = 1;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
        }
        telemetry.addLine(title() + (parkOnly() ? "" : "   (D-pad left = plan A, right = plan B)"));
        telemetry.addLine("Keep the robot STILL during INIT (odometry calibrating), then press START (play).");
        telemetry.addLine("Then: drive to the START spot and press SQUARE.");
        showSaved();
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);   // true = motors brake when you let go
    }

    @Override
    public void loop() {
        follower.update();

        // same stick feel as TeleOp: left stick as one arrow, any push past the dead zone moves the robot a little
        double scale = Math.max(gamepad1.left_trigger, gamepad1.right_trigger) > 0.5 ? SLOW_SCALE : DRIVE_SCALE;
        double fwd = -gamepad1.left_stick_y, left = -gamepad1.left_stick_x;
        double push = Math.hypot(fwd, left);
        double movePower = drivetrain.shape(push, scale, drivetrain.MIN_DRIVE_POWER);
        double fwdCmd  = push > 0 ? fwd / push * movePower : 0;
        double leftCmd = push > 0 ? left / push * movePower : 0;
        double turnCmd = -drivetrain.shape(gamepad1.right_stick_x, scale, drivetrain.MIN_TURN_POWER);
        follower.setTeleOpDrive(fwdCmd, leftCmd, turnCmd, true);

        Pose p = follower.getPose();
        checkOdometry(p, fwdCmd, leftCmd, turnCmd);

        // 1. START
        if (gamepad1.square && !prevSquare) {
            startPose = p;
            shootSaved = endSaved = false;
            legPoints.clear();
            gamepad1.rumble(200);
            RobotLog.ii("SETUP", "%s %s START set (odometry check: %s, battery %.1f V)",
                    alliance(), parkOnly() ? "PARK" : "PLAN " + autoSpots.planName(autoSpots.plan),
                    odometryText(), hwManager.batteryVoltage());
            message = parkOnly() ? "START set. Now drive into the LOADING ZONE (not touching the wall) and press O."
                    : "START set. Now drive to where it should shoot and press X (or X right here to shoot from the start).";
        }
        // path points (only after START): L1 = drive through here, D-pad down = undo the last one
        if (startPose != null && gamepad1.left_bumper && !prevL1) {
            if (legPoints.size() >= autoSpots.MAX_POINTS) {
                message = "!! " + autoSpots.MAX_POINTS + " path points is the most per route";
                gamepad1.rumble(800);
            } else {
                double[] r = relative(p);
                legPoints.add(new double[] {Math.round(r[0]), Math.round(r[1]), Math.round(r[2])});
                message = "Path point " + legPoints.size() + " added: " + autoSpots.text(legPoints.get(legPoints.size() - 1));
                gamepad1.rumble(150);
            }
        }
        if (gamepad1.dpad_down && !prevDown && !legPoints.isEmpty()) {
            legPoints.remove(legPoints.size() - 1);
            message = "Last path point removed (" + legPoints.size() + " left)";
        }
        prevL1 = gamepad1.left_bumper;
        prevDown = gamepad1.dpad_down;

        // 2. SHOOT and 3. LOADING ZONE (only after START)
        boolean cross = gamepad1.cross && !prevCross, circle = gamepad1.circle && !prevCircle;
        if ((cross && !parkOnly()) || circle) {
            if (startPose == null) {
                message = "!! First put the robot on its START spot and press SQUARE.";
                gamepad1.rumble(800);
            } else if (parkOnly()) {
                autoSpots.setPoints(autoSpots.parkRoute(alliance()), legPoints);
                message = save(autoSpots.park(alliance()), p, "SAFE PARK") + viaText();
                legPoints.clear();
                endSaved = true;
            } else if (cross) {
                int plan = autoSpots.plan;
                autoSpots.setPoints(autoSpots.route(alliance(), plan, true), legPoints);
                message = save(autoSpots.shoot(alliance(), plan), p, "PLAN " + autoSpots.planName(plan) + " SHOOT") + viaText();
                if (!autoSpots.isSet(autoSpots.shoot(alliance(), plan))) message += " (= START spot: shoots without driving)";
                legPoints.clear();
                shootSaved = true;
            } else {
                int plan = autoSpots.plan;
                autoSpots.setPoints(autoSpots.route(alliance(), plan, false), legPoints);
                message = save(autoSpots.end(alliance(), plan), p, "PLAN " + autoSpots.planName(plan) + " LOADING ZONE") + viaText();
                legPoints.clear();
                endSaved = true;
            }
        }
        prevSquare = gamepad1.square;
        prevCross = gamepad1.cross;
        prevCircle = gamepad1.circle;

        // hold triangle to test-shoot from here (shooting auto setup only)
        if (gamepad1.triangle && !parkOnly()) {
            if (!prevTriangle) burst.start();
            burst.update();
        } else if (prevTriangle) {
            burst.stop();
        }
        prevTriangle = gamepad1.triangle;

        // hold R1 to load balls again (only when not shooting)
        boolean wantLoad = gamepad1.right_bumper && !gamepad1.triangle;
        if (wantLoad != loading) {
            loading = wantLoad;
            if (loading) intake.turnOn();
            else         intake.stop();
        }

        // ---- screen ----
        telemetry.addLine(title());
        telemetry.addLine((startPose != null ? "[OK] " : "[  ] ") + "SQUARE = 1. START here (robot on its start spot, touching the wall)");
        if (!parkOnly()) {
            telemetry.addLine((shootSaved ? "[OK] " : "[  ] ") + "X = 2. SHOOT here   (hold TRIANGLE to test the aim, R1 = load)");
        }
        telemetry.addLine((endSaved ? "[OK] " : "[  ] ") + "O = " + (parkOnly() ? "2" : "3")
                + ". LOADING ZONE here (partly in it, NOT touching the wall)");
        telemetry.addLine("     L1 = path point here (to drive around things), D-pad down = undo   points on this route: "
                + legPoints.size());
        if (startPose != null) {
            double[] r = relative(p);
            telemetry.addData("From START", "%.0f in ahead, %.0f in left, turned %.0f deg", r[0], r[1], r[2]);
        } else {
            telemetry.addData("From START", "START not set yet - press SQUARE on the start spot");
        }
        telemetry.addData("Odometry check", odometryText());
        double volts = hwManager.batteryVoltage();
        if (volts < 12.5) telemetry.addData("!! Battery", "%.1f V - LOW: motors may cut out, charge or swap", volts);
        if (gamepad1.triangle) {
            telemetry.addData("Shooter", "%.0f / %d  balls: %d", hwManager.shooterVelocity(), baseTeleOp.SHOT_VELOCITY, burst.balls());
        }
        if (!message.isEmpty()) telemetry.addLine(message);
        showSaved();
        telemetry.addLine("Rules: stay on your side (G402), don't touch the HIVE (G417).");
        telemetry.update();
    }

    private String viaText() {
        return legPoints.isEmpty() ? " (straight line)" : " (via " + legPoints.size() + " path point" + (legPoints.size() > 1 ? "s)" : ")");
    }

    private String title() {
        return parkOnly() ? "SETUP SAFE PARK " + alliance()
                : "SETUP SHOOT AUTO " + alliance() + "   PLAN " + autoSpots.planName(autoSpots.plan);
    }

    // where the robot is now, measured from the START (□) spot: {inches ahead, inches left, degrees turned left}
    private double[] relative(Pose p) {
        double dx = p.getX() - startPose.getX(), dy = p.getY() - startPose.getY(), h0 = startPose.getHeading();
        double ahead = dx * Math.cos(h0) + dy * Math.sin(h0);
        double left = -dx * Math.sin(h0) + dy * Math.cos(h0);
        double turn = Math.toDegrees(Math.atan2(Math.sin(p.getHeading() - h0), Math.cos(p.getHeading() - h0)));
        return new double[] {ahead, left, turn};
    }

    // Odometry check: while you drive, the position must change the way the sticks say. Driving FORWARD must make
    // "ahead" go UP, driving LEFT must make "left" go UP, turning LEFT must make "turned" go UP. Each score adds
    // up how far the robot moved in the direction you asked (negative = the odometry counts the wrong way).
    private Pose lastPose = null;
    private double fwdScore, leftScore, turnScore;
    private static final double CHECK_IN = 6, CHECK_RAD = Math.toRadians(20);
    private boolean odometryBad = false;

    private void checkOdometry(Pose p, double fwdCmd, double leftCmd, double turnCmd) {
        if (lastPose != null) {
            double dx = p.getX() - lastPose.getX(), dy = p.getY() - lastPose.getY(), h = lastPose.getHeading();
            double movedFwd  =  dx * Math.cos(h) + dy * Math.sin(h);   // in the robot's own directions
            double movedLeft = -dx * Math.sin(h) + dy * Math.cos(h);
            double turned = Math.atan2(Math.sin(p.getHeading() - h), Math.cos(p.getHeading() - h));
            if (Math.abs(fwdCmd) > 0.04 && Math.abs(fwdCmd) > 2 * Math.abs(leftCmd))  fwdScore  += Math.signum(fwdCmd) * movedFwd;
            if (Math.abs(leftCmd) > 0.04 && Math.abs(leftCmd) > 2 * Math.abs(fwdCmd)) leftScore += Math.signum(leftCmd) * movedLeft;
            if (Math.abs(turnCmd) > 0.04 && Math.abs(fwdCmd) + Math.abs(leftCmd) < 0.02) turnScore += Math.signum(turnCmd) * turned;
            boolean wasBad = odometryBad;
            odometryBad = fwdScore < -CHECK_IN || leftScore < -CHECK_IN || turnScore < -CHECK_RAD;
            if (odometryBad && !wasBad) {
                RobotLog.ww("SETUP", "ODOMETRY WRONG WAY: %s", odometryText());
                gamepad1.rumble(1000);
            }
        }
        lastPose = p;
    }

    private String odometryText() {
        return "forward " + check(fwdScore, CHECK_IN) + ", left " + check(leftScore, CHECK_IN)
                + ", turn left " + check(turnScore, CHECK_RAD)
                + (odometryBad ? "  !! COUNTS THE WRONG WAY - don't save spots, tell the coder" : "");
    }

    private static String check(double score, double needed) {
        if (score > needed) return "OK";
        if (score < -needed) return "WRONG WAY";
        return "drive to check";
    }

    private String save(double[] spot, Pose p, String name) {
        double[] r = relative(p);
        spot[0] = Math.round(r[0]);
        spot[1] = Math.round(r[1]);
        spot[2] = Math.round(r[2]);
        double far = Math.hypot(spot[0], spot[1]);
        RobotLog.ii("SETUP", "%s %s saved: %.0f ahead, %.0f left, %.0f deg from START (odometry check: %s, battery %.1f V)",
                alliance(), name, spot[0], spot[1], spot[2], odometryText(), hwManager.batteryVoltage());
        boolean ok = settings.save();
        gamepad1.rumble(ok ? 200 : 800);
        String msg = ok ? name + " SAVED: " + autoSpots.text(spot) + " from START" : name + " could NOT be saved";
        if (far > FAR_IN) msg += String.format("  !! that is %.0f ft from START - was SQUARE pressed on the start spot?", far / 12);
        if (odometryBad) msg += "  !! odometry counts the wrong way - this spot is wrong";
        return msg;
    }

    private void showSaved() {
        if (parkOnly()) {
            telemetry.addData("Saved safe park spot", autoSpots.text(autoSpots.park(alliance())));
            return;
        }
        int plan = autoSpots.plan;
        telemetry.addData("Saved plan " + autoSpots.planName(plan) + " shoot", autoSpots.shootText(alliance(), plan));
        telemetry.addData("Saved plan " + autoSpots.planName(plan) + " loading zone", autoSpots.text(autoSpots.end(alliance(), plan)));
    }

    @Override
    public void stop() {
        burst.stop();
        intake.stop();
        follower.breakFollowing();
    }
}
