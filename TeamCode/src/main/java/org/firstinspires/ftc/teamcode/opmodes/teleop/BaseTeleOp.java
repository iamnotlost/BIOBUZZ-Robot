package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.HeadingHold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequencer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.MatchTimer;
import org.firstinspires.ftc.teamcode.util.Settings;

/**
 * Driver control for both alliances (RedTeleOp / BlueTeleOp only pick the alliance).
 *
 * <pre>
 * Gamepad 1                                      Gamepad 2
 *   left stick      drive                          D-pad up/down     intake speed
 *   right stick X   turn                           D-pad left/right  shot speed -/+ 5
 *   triggers        slow down (full = 25%)         R1 / L1           drive top speed +/-
 *   click L stick   robot / field-centric          cross             save all settings
 *   click R stick   top speed 40-60-80-100%
 *   options         reset heading
 *   share           heading hold on/off
 *   hold R1         load (intake)
 *   hold cross      feeder pushes balls up
 *   hold circle     everything backwards (unstick)
 *   hold triangle   shoot (ShotSequencer)
 *   hold L1         pre-spin the shooter
 *   D-pad up/down   shot speed +/- 5 (saved)
 * </pre>
 */
public abstract class BaseTeleOp extends OpMode {
    // Start-up choices, saved in Settings
    public static boolean FIELD_CENTRIC_DEFAULT = false;
    public static boolean HEADING_HOLD_DEFAULT = false;
    public static double MIN_TRIGGER_SPEED = 0.25;   // speed with a trigger squeezed all the way
    public static final int SHOT_SPEED_STEP = 5;      // small changes decide rim / in / bounce out

    protected abstract Alliance alliance();

    private RobotHardware hw;
    private Drivetrain drivetrain;
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;
    private ShotSequencer shots;
    private HeadingHold headingHold;
    private final MatchTimer matchTimer = new MatchTimer();

    private boolean fieldCentric, readyRumbled, opponentNectar;
    private String shooterStatus = "", ballStatus = "", savedMessage = "";
    private final ElapsedTime savedMessageTimer = new ElapsedTime();

    @Override
    public void init() {
        boolean loaded = Settings.load();
        hw = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hw);
        intake = new Intake(hw);
        feeder = new Feeder(hw);
        shooter = new Shooter(hw);
        shots = new ShotSequencer(hw, shooter, feeder, intake);
        headingHold = new HeadingHold(HEADING_HOLD_DEFAULT);
        fieldCentric = FIELD_CENTRIC_DEFAULT;

        telemetry.addData("Init", "complete (%s)", alliance());
        telemetry.addData("Settings", loaded ? "loaded" : "could not read the file - using defaults");
        telemetry.addData("!!", "Keep the robot STILL during INIT (the Pinpoint calibrates)");
        telemetry.addData("Missing", hw.missingText());
    }

    @Override
    public void start() {
        matchTimer.start();
    }

    @Override
    public void loop() {
        handleHeadingButtons();
        // The Pinpoint is only read when something uses the heading (it is an extra I2C read)
        boolean needHeading = fieldCentric || headingHold.enabled();
        hw.update(needHeading ? RobotHardware.Odometry.HEADING : RobotHardware.Odometry.NONE);
        if (!hw.hasHeading()) {   // Pinpoint lost (just now?) and no IMU: only robot-centric driving works
            fieldCentric = false;
            headingHold.disable();
            needHeading = false;
        }
        matchTimer.update(gamepad1, gamepad2);

        if (matchTimer.isOver()) {
            stopAll();
            telemetry.addData("Match", "OVER - press STOP");
            telemetry.update();
            return;
        }

        boolean nectar = hw.elementSensor.isOpponentNectar(alliance());
        if (nectar && !opponentNectar) gamepad1.rumble(500);
        opponentNectar = nectar;

        drive(needHeading ? hw.heading() : 0);
        handleSettingButtons();
        handleBalls();
        shooter.update();
        showDriverScreen();
    }

    @Override
    public void stop() {
        stopAll();
    }

    // ---------------- driving ----------------

    private void handleHeadingButtons() {
        if (gamepad1.leftStickButtonWasPressed() && hw.hasHeading()) fieldCentric = !fieldCentric;
        if (gamepad1.shareWasPressed() && hw.hasHeading()) headingHold.toggle();
        if (gamepad1.optionsWasPressed()) {
            hw.resetHeading();
            headingHold.reset();
        }
    }

    private void drive(double heading) {
        // a trigger lowers only the top speed, so small pushes still move the robot while slowed down
        double speed = 1.0 - (1.0 - MIN_TRIGGER_SPEED) * Math.max(gamepad1.left_trigger, gamepad1.right_trigger);

        // left stick as one arrow: dead zone and feel act on how far it is pushed, the direction is kept exactly,
        // so pushing straight ahead never drifts sideways
        double sx = -gamepad1.left_stick_x, sy = gamepad1.left_stick_y;
        double push = Math.hypot(sx, sy);
        double power = Drivetrain.shape(push, Drivetrain.MAX_SPEED * speed, Drivetrain.MIN_DRIVE_POWER);
        double x = push > 0 ? sx / push * power : 0;
        double y = push > 0 ? sy / push * power : 0;
        double turn = -Drivetrain.shape(gamepad1.right_stick_x, Drivetrain.TURN_SPEED * speed, Drivetrain.MIN_TURN_POWER);
        turn = headingHold.apply(turn, gamepad1.right_stick_x, heading);

        drivetrain.drive(x, y, turn, fieldCentric ? heading : 0);
    }

    // ---------------- live tuning ----------------

    private void handleSettingButtons() {
        if (gamepad1.dpadUpWasPressed()) changeShotSpeed(SHOT_SPEED_STEP);
        if (gamepad1.dpadDownWasPressed()) changeShotSpeed(-SHOT_SPEED_STEP);
        if (gamepad1.rightStickButtonWasPressed()) {
            gamepad1.rumbleBlips(Drivetrain.nextSpeedLevel());   // 1-4 buzzes = speed level 1-4
            saveSettings();
        }

        if (gamepad2.dpadUpWasPressed()) Intake.adjustSpeed(1);
        if (gamepad2.dpadDownWasPressed()) Intake.adjustSpeed(-1);
        if (gamepad2.dpadRightWasPressed()) changeShotSpeed(SHOT_SPEED_STEP);
        if (gamepad2.dpadLeftWasPressed()) changeShotSpeed(-SHOT_SPEED_STEP);
        if (gamepad2.rightBumperWasPressed()) { Drivetrain.adjustMaxSpeed(1); saveSettings(); }
        if (gamepad2.leftBumperWasPressed()) { Drivetrain.adjustMaxSpeed(-1); saveSettings(); }
        if (gamepad2.crossWasPressed()) gamepad2.rumble(saveSettings() ? 200 : 800);
    }

    private void changeShotSpeed(int step) {
        ShotSequencer.SHOT_VELOCITY = Math.max(200, Math.min(2800, ShotSequencer.SHOT_VELOCITY + step));
        if (!saveSettings()) gamepad1.rumble(800);   // long buzz: could not save
    }

    private boolean saveSettings() {
        boolean ok = Settings.save();
        savedMessage = ok ? "SAVED" : "SAVE FAILED";
        savedMessageTimer.reset();
        return ok;
    }

    // ---------------- intake, feeder, shooter ----------------

    // Every ball button works while HELD. Shooting wins over everything; circle (backwards) wins over R1 and cross.
    private void handleBalls() {
        if (gamepad1.triangle) {
            if (!shots.active()) shots.start();
            shots.update();
            if (shots.firstBallNow()) gamepad1.rumble(200);   // short buzz: the first ball is going in
            shooterStatus = shots.status() + "  (let go of triangle to stop)";
            ballStatus = "going into the shooter";
            return;
        }
        if (shots.active()) shots.end();   // the wheel is set just below (pre-spin keeps it going)

        boolean reverse = gamepad1.circle;
        boolean load = gamepad1.right_bumper && !reverse;
        boolean pushUp = gamepad1.cross && !reverse;
        boolean preSpin = gamepad1.left_bumper && !reverse;

        if (reverse) {
            // everything backwards, shooter wheel too, so a ball stuck anywhere comes back out
            intake.out();
            feeder.run(-Feeder.REVERSE_VELOCITY);
            shooter.reverse();
        } else {
            if (preSpin) shooter.spinTo(ShotSequencer.SHOT_VELOCITY);
            else shooter.turnOff();
            if (load) intake.load();
            else intake.stop();
            // while loading the feeder stays still so a ball rolls in and settles where the feeder grips it
            if (pushUp) feeder.run(Feeder.UP_VELOCITY);
            else feeder.stop();
        }

        if (preSpin) {
            double vel = hw.shooterVelocity();
            boolean ready = Math.abs(vel - ShotSequencer.SHOT_VELOCITY) <= ShotSequencer.READY_TOLERANCE;
            if (ready && !readyRumbled) gamepad1.rumble(200);   // short buzz: wheel is up to speed
            readyRumbled |= ready;
            shooterStatus = ready ? "READY - hold triangle"
                    : String.format("pre-spinning %.0f / %d", vel, ShotSequencer.SHOT_VELOCITY);
        } else {
            readyRumbled = false;
            shooterStatus = "off";
        }
        ballStatus = reverse ? "BACKWARDS" : pushUp ? "pushing up"
                : load ? (intake.unjamming() ? "unjamming..." : "loading") : "stopped (hold R1 to load)";
    }

    // G404: nothing powered once TELEOP is over
    private void stopAll() {
        drivetrain.stop();
        shots.stop();
    }

    // ---------------- driver screen ----------------

    private void showDriverScreen() {
        String time = matchTimer.display();
        if (matchTimer.parkTime()) time += "   GO PARK in the LOADING ZONE";
        else if (matchTimer.nectarFlowerAllowed()) time += "   NECTAR into FLOWERS allowed";
        telemetry.addData("Time", time);
        telemetry.addData("Shooter", shooterStatus);
        telemetry.addData("Balls", ballStatus);
        telemetry.addData("Shot speed", "%d  (D-pad up/down: stronger/weaker) %s", ShotSequencer.SHOT_VELOCITY,
                savedMessageTimer.seconds() < 3 && !savedMessage.isEmpty() ? "- " + savedMessage : "");
        telemetry.addData("Intake speed", "%d  (gamepad 2 D-pad up/down)", Intake.IN_VELOCITY);
        telemetry.addData("Battery", "%.1f V", hw.batteryVoltage());
        if (hw.pinpointFailed()) telemetry.addData("!!", "Pinpoint lost - check its cable. Robot-centric only until restart");
        if (opponentNectar) telemetry.addData("!!", "OPPONENT NECTAR - hold circle to spit it out");
        telemetry.addData("Driving", "%s, top speed %.0f%%  (L-stick click: mode, R-stick click: speed)",
                fieldCentric ? "FIELD-centric" : "ROBOT-centric", Drivetrain.MAX_SPEED * 100);
        telemetry.addData("Heading hold", headingHold.enabled() ? "on (Share)" : headingHold.gaveUp()
                ? "OFF - it had to turn too far, heading reading looks wrong (Share to retry)" : "off (Share)");
        if (!hw.missing.isEmpty()) telemetry.addData("Not found", hw.missingText());
        telemetry.update();
    }
}
