package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Teach the shooting auto where to go: put the robot on its START spot, INIT, then drive it by hand.
//   Left stick / right stick   drive / turn (half speed; hold a trigger for quarter speed)
//   Cross (X)                  SAVE this as the SHOOT spot
//   Circle (O)                 SAVE this as the END spot (in your LOADING ZONE, NOT touching the wall)
//   Hold triangle              shoot from here, to check the aim (same burst as the auto)
//   Hold R1                    intake, to load balls again for another test
// Positions come from the same odometry the auto uses, measured from where the robot was at INIT.
public abstract class baseSetupAuto extends OpMode {
    protected abstract Alliance alliance();

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

    @Override
    public void init_loop() {
        follower.update();
        telemetry.addLine("SETUP AUTO " + alliance());
        telemetry.addLine("1. Robot on its START spot: touching the wall, on your side, NOT in the LOADING ZONE.");
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

        double scale = Math.max(gamepad1.left_trigger, gamepad1.right_trigger) > 0.5 ? SLOW_SCALE : DRIVE_SCALE;
        follower.setTeleOpDrive(
                -drivetrain.curve(gamepad1.left_stick_y) * scale,
                -drivetrain.curve(gamepad1.left_stick_x) * scale,
                -drivetrain.curve(gamepad1.right_stick_x) * scale,
                true);

        Pose p = follower.getPose();
        double headingDeg = Math.toDegrees(Math.atan2(Math.sin(p.getHeading()), Math.cos(p.getHeading())));

        if (gamepad1.cross && !prevCross)   message = save(autoSpots.shoot(alliance()), p, headingDeg, "SHOOT");
        if (gamepad1.circle && !prevCircle) message = save(autoSpots.end(alliance()), p, headingDeg, "END");
        prevCross = gamepad1.cross;
        prevCircle = gamepad1.circle;

        // hold triangle to test-shoot from here
        if (gamepad1.triangle) {
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

        telemetry.addLine("X = save SHOOT spot   O = save END spot   hold TRIANGLE = test shot   hold R1 = load");
        telemetry.addData("Now", "%.0f in ahead, %.0f in left, turned %.0f deg", p.getX(), p.getY(), headingDeg);
        if (gamepad1.triangle) {
            telemetry.addData("Shooter", "%.0f / %d  balls: %d", hwManager.shooterVelocity(), baseTeleOp.SHOT_VELOCITY, burst.balls());
        }
        if (!message.isEmpty()) telemetry.addLine(message);
        showSpots();
        telemetry.addLine("Rules: END spot partly in your LOADING ZONE and NOT touching the wall = LEAVE 3 + PARK 5."
                + " Stay on your side of the field (G402) and don't touch the HIVE (G417).");
        telemetry.update();
    }

    private String save(double[] spot, Pose p, double headingDeg, String name) {
        spot[0] = Math.round(p.getX());
        spot[1] = Math.round(p.getY());
        spot[2] = Math.round(headingDeg);
        boolean ok = settings.save();
        gamepad1.rumble(ok ? 200 : 800);
        return ok ? name + " spot SAVED on the robot" : name + " spot could NOT be saved";
    }

    private void showSpots() {
        telemetry.addData("Shoot spot", autoSpots.text(autoSpots.shoot(alliance())));
        telemetry.addData("End spot", autoSpots.text(autoSpots.end(alliance())));
    }

    @Override
    public void stop() {
        burst.stop();
        intake.stop();
        follower.breakFollowing();
    }
}
