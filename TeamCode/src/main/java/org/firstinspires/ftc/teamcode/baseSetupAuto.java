package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Teach the shooting auto where to go: put the robot on its START spot, INIT, then drive it by hand.
//   Left stick / right stick   drive / turn (half speed; hold a trigger for quarter speed)
//   Cross (X)                  SAVE this as the SHOOT spot
//   Circle (O)                 SAVE this as the END spot (in your LOADING ZONE, NOT touching the wall)
//   Hold triangle              shoot from here, to check the aim (same burst as the auto)
//   Hold R1                    intake, to load balls again for another test
// Positions come from the same odometry the auto uses, measured from where the robot was at INIT.
// Park-only version (setupParkRed / setupParkBlue): start on the CORNER start spot, drive into the LOADING ZONE,
// Circle (O) saves the SAFE PARK spot used by parkRed / parkBlue. Cross and triangle do nothing there.
public abstract class baseSetupAuto extends OpMode {
    protected abstract Alliance alliance();

    // true = teach the safe park auto (parkRed / parkBlue) instead of the shooting auto
    protected boolean parkOnly() {
        return false;
    }

    public static double DRIVE_SCALE = 0.5;
    public static double SLOW_SCALE  = 0.25;

    hwManager hwManager;
    shooter shooter;
    supply supply;
    intake intake;
    burst burst;
    Follower follower;

    private boolean prevCross, prevCircle, prevTriangle;
    private boolean loading = false;
    private String message = "";

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

    private boolean prevLeft, prevRight;

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
        telemetry.addLine(parkOnly() ? "SETUP SAFE PARK " + alliance()
                : "SETUP SHOOT AUTO " + alliance() + "   PLAN " + autoSpots.planName(autoSpots.plan)
                  + "   (D-pad left = A, right = B)");
        telemetry.addLine("1. Robot on its " + (parkOnly() ? "CORNER " : "plan " + autoSpots.planName(autoSpots.plan) + " ")
                + "START spot: touching the wall, on your side, NOT in the LOADING ZONE.");
        telemetry.addLine("2. Keep it STILL now (odometry calibrating), then press START.");
        telemetry.addLine("Mark the start spot with tape: the auto must start exactly here every match.");
        showSpots();
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
        double headingDeg = Math.toDegrees(Math.atan2(Math.sin(p.getHeading()), Math.cos(p.getHeading())));
        checkOdometry(p, fwdCmd, leftCmd, turnCmd);

        if (parkOnly()) {
            if (gamepad1.circle && !prevCircle) message = save(autoSpots.park(alliance()), p, headingDeg, "SAFE PARK");
        } else {
            int plan = autoSpots.plan;
            if (gamepad1.cross && !prevCross) {
                message = save(autoSpots.shoot(alliance(), plan), p, headingDeg, "PLAN " + autoSpots.planName(plan) + " SHOOT");
                if (!autoSpots.isSet(autoSpots.shoot(alliance(), plan))) message += " (= START spot: shoots without driving)";
            }
            if (gamepad1.circle && !prevCircle) {
                message = save(autoSpots.end(alliance(), plan), p, headingDeg, "PLAN " + autoSpots.planName(plan) + " END");
            }
        }
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

        telemetry.addLine(parkOnly() ? "Drive into your LOADING ZONE (not touching the wall), then O = save SAFE PARK spot"
                : "X = save SHOOT spot   O = save END spot   hold TRIANGLE = test shot   hold R1 = load");
        telemetry.addData("Now", "%.0f in ahead, %.0f in left, turned %.0f deg", p.getX(), p.getY(), headingDeg);
        telemetry.addData("Odometry check", odometryText());
        double volts = hwManager.batteryVoltage();
        if (volts < 12.5) telemetry.addData("!! Battery", "%.1f V - LOW: motors may cut out, charge or swap", volts);
        if (gamepad1.triangle) {
            telemetry.addData("Shooter", "%.0f / %d  balls: %d", hwManager.shooterVelocity(), baseTeleOp.SHOT_VELOCITY, burst.balls());
        }
        if (!message.isEmpty()) telemetry.addLine(message);
        showSpots();
        telemetry.addLine("Rules: " + (parkOnly() ? "PARK" : "END")
                + " spot partly in your LOADING ZONE and NOT touching the wall = LEAVE 3 + PARK 5."
                + " Stay on your side of the field (G402) and don't touch the HIVE (G417).");
        telemetry.update();
    }

    // Odometry check: while you drive, the position must change the way the sticks say. Driving FORWARD must make
    // "ahead" go UP, driving LEFT must make "left" go UP, turning LEFT must make "turned" go UP. Each score adds
    // up how far the robot moved in the direction you asked (negative = the odometry counts the wrong way).
    private Pose lastPose = null;
    private double fwdScore, leftScore, turnScore;
    private static final double CHECK_IN = 6, CHECK_RAD = Math.toRadians(20);

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

    private boolean odometryBad = false;

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

    private String save(double[] spot, Pose p, double headingDeg, String name) {
        spot[0] = Math.round(p.getX());
        spot[1] = Math.round(p.getY());
        spot[2] = Math.round(headingDeg);
        RobotLog.ii("SETUP", "%s %s saved: %.0f ahead, %.0f left, %.0f deg (odometry check: %s, battery %.1f V)",
                alliance(), name, spot[0], spot[1], spot[2], odometryText(), hwManager.batteryVoltage());
        boolean ok = settings.save();
        gamepad1.rumble(ok ? 200 : 800);
        String msg = ok ? name + " spot SAVED on the robot" : name + " spot could NOT be saved";
        // (a negative "ahead" is fine: it just means the robot drove backwards, e.g. it starts facing the wall)
        if (odometryBad) msg += "  !! odometry counts the wrong way - this spot is wrong";
        return msg;
    }

    private void showSpots() {
        if (parkOnly()) {
            telemetry.addData("Safe park spot", autoSpots.text(autoSpots.park(alliance())));
            return;
        }
        int plan = autoSpots.plan;
        telemetry.addData("Plan " + autoSpots.planName(plan) + " shoot spot", autoSpots.shootText(alliance(), plan));
        telemetry.addData("Plan " + autoSpots.planName(plan) + " end spot", autoSpots.text(autoSpots.end(alliance(), plan)));
    }

    @Override
    public void stop() {
        burst.stop();
        intake.stop();
        follower.breakFollowing();
    }
}
