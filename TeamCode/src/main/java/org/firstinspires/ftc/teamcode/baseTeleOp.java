package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


// Shared TeleOp for both alliances; redTeleOp / blueTeleOp only pick the Alliance
public abstract class baseTeleOp extends OpMode {
    hwManager hwManager;
    drivetrain drivetrain;
    supply supply;
    shooter shooter;
    intake intake;
    matchTimer matchTimer = new matchTimer();

    protected abstract Alliance alliance();

    boolean prevDPadUp, prevDPadDown, prevG2Up, prevG2Down;
    boolean prevRBumper, prevLBumper;
    boolean prevDPadRight, prevDPadLeft;
    boolean prevOptions;
    boolean prevTriangle, prevSquare, prevCross, prevCircle;

    // Shooting: tap triangle (close) or square (far) to start, tap again to stop. The wheel spins up
    // first and the feeder only pushes a ball in while the wheel is at speed, so every ball leaves at
    // full speed.
    public static int CLOSE_SHOT_VELOCITY = 1320;
    public static int FAR_SHOT_VELOCITY   = 1860;
    public static int SHOT_FEED_VELOCITY  = 1500;
    // Feeder speed for cross (push balls up) and circle (backwards)
    public static int FEED_UP_VELOCITY    = 1400;
    public static int REVERSE_VELOCITY    = 800;
    private static final double READY_TOLERANCE = 80;   // within this of the target = ready to feed
    private static final double DROP_TOLERANCE  = 150;  // this far below = a ball just went through, wait

    private static final int NOT_SHOOTING = 0, SHOOT_CLOSE = 1, SHOOT_FAR = 2;
    int shootMode = NOT_SHOOTING; // triangle / square
    boolean loading = false;      // R1
    boolean preSpin = false;      // L1
    boolean feedingUp = false;    // cross
    boolean reversing = false;    // circle
    boolean feedingShot = false;
    // last speeds sent, so motors are only told when something changes
    private int lastFeeder = 0;
    private int lastIntake = 0;   // 1 in, -1 out, 0 stopped
    boolean buttonPressed = false;
    boolean prevOpponentNectar = false;
    
    boolean autoRotating = false;
    // Red angles; blue mirrors them via Alliance.sign
    private static final double TARGET_HEADING_FAR_DEG = 68.5;
    private static final double TARGET_HEADING_45_DEG  = 45;
    private double targetHeadingFarRad;
    private double targetHeading45Rad;
    private double currentTargetHeading;
    private static final double HEADING_KP         = 1.2;
    private static final double HEADING_TOLERANCE  = Math.toRadians(2);
    private static final double MAX_ROT_POWER      = 0.5;
    private static final double MIN_ROT_POWER      = 0.15;
    private static final double CANCEL_STICK_THRESH = 0.15;

    // Heading hold: after you let go of the turn stick, gently corrects slow drift. Share toggles it.
    private boolean headingHoldEnabled = true;
    private boolean headingHoldActive = false;
    private double  headingHoldRad    = 0;
    private boolean prevShare;
    private final ElapsedTime headingHoldDelayTimer = new ElapsedTime();
    private static final long   HEADING_HOLD_DELAY_MS = 400;
    private static final double HOLD_KP          = 0.5;
    private static final double HOLD_MAX_POWER   = 0.15;
    private static final double HOLD_TOLERANCE   = Math.toRadians(3);

    @Override
    public void init() {
        hwManager = new hwManager();
        hwManager.initHardware(hardwareMap);

        drivetrain = new drivetrain(hwManager);
        supply = new supply(hwManager);
        shooter = new shooter(hwManager);
        intake = new intake(hwManager);

        targetHeadingFarRad  = Math.toRadians(alliance().sign * TARGET_HEADING_FAR_DEG);
        targetHeading45Rad   = Math.toRadians(alliance().sign * TARGET_HEADING_45_DEG);
        currentTargetHeading = targetHeadingFarRad;

        telemetry.addData("init", "complete (" + alliance() + ")");
        telemetry.addData("Missing", hwManager.missingText());
    }

    @Override
    public void start() {
        matchTimer.start();
    }

    @Override
    public void loop() {
        hwManager.update();
        matchTimer.update(gamepad1, gamepad2);

        if (matchTimer.isOver()) {
            stopAll();
            telemetry.addData("Match", "OVER - press STOP");
            telemetry.update();
            return;
        }

        shooter.update();
        buttonPressed = hwManager.buttonPressed();

        telemetry.addData("Time left", matchTimer.display());
        telemetry.addData("NECTAR -> FLOWER", matchTimer.nectarFlowerAllowed() ? "ALLOWED" : "not yet");
        boolean opponentNectar = hwManager.elementSensor.isOpponentNectar(alliance());
        if (opponentNectar && !prevOpponentNectar) gamepad1.rumble(500);
        prevOpponentNectar = opponentNectar;

        telemetry.addData("Ball", opponentNectar ? "OPPONENT NECTAR - spit it out!" : hwManager.elementSensor.read());
        telemetry.addData("Drive top speed", "%.0f%%  (gamepad 1 D-pad up/down)", drivetrain.MAX_SPEED * 100);
        telemetry.addData("Intake speed", "%d  (gamepad 2 D-pad up/down)", intake.IN_VELOCITY);
        telemetry.addData("Missing", hwManager.missingText());
        telemetry.addData("Button Pressed", buttonPressed);
        telemetry.addData("Shooter velocity", hwManager.shooterVelocity());
        telemetry.addData("Heading (deg)", Math.toDegrees(hwManager.getHeading()));
        telemetry.addData("Auto-rotate", autoRotating ? "ON (target " + Math.toDegrees(currentTargetHeading) + " deg)" : "off");
        telemetry.addData("Heading hold", !headingHoldEnabled ? "OFF (Share to turn on)"
                : headingHoldActive ? String.format("holding %.0f deg (Share to turn off)", Math.toDegrees(headingHoldRad)) : "on");

        double x  = -drivetrain.curve(gamepad1.left_stick_x)  * drivetrain.MAX_SPEED;
        double y  =  drivetrain.curve(gamepad1.left_stick_y)  * drivetrain.MAX_SPEED;
        double rx = -drivetrain.curve(gamepad1.right_stick_x) * drivetrain.TURN_SPEED;

        boolean slowMode  = gamepad1.right_trigger > 0.05;
        boolean superSlow = gamepad1.left_trigger  > 0.05;

        double speedMult = superSlow ? 0.25 : slowMode ? 0.5 : 1.0;

        if (autoRotating && Math.abs(gamepad1.right_stick_x) > CANCEL_STICK_THRESH) {
            autoRotating = false;
        }

        double driveRx = rx * speedMult;
        if (autoRotating) {
            double error = normalizeAngle(currentTargetHeading - hwManager.getHeading());
            if (Math.abs(error) < HEADING_TOLERANCE) {
                autoRotating = false;
                driveRx = 0;
                headingHoldRad    = currentTargetHeading;
                headingHoldActive = true;
            } else {
                double p = HEADING_KP * error;
                if (Math.abs(p) > MAX_ROT_POWER) p = Math.signum(p) * MAX_ROT_POWER;
                else if (Math.abs(p) < MIN_ROT_POWER) p = Math.signum(p) * MIN_ROT_POWER;
                driveRx = p;
            }
        } else if (Math.abs(gamepad1.right_stick_x) > CANCEL_STICK_THRESH) {
            headingHoldActive = false;
            headingHoldDelayTimer.reset();
        } else {
            if (headingHoldEnabled && !headingHoldActive && headingHoldDelayTimer.milliseconds() >= HEADING_HOLD_DELAY_MS) {
                headingHoldRad    = hwManager.getHeading();
                headingHoldActive = true;
            }
            if (headingHoldActive) {
                // Gentle: small power, small dead zone, so it nudges drift back instead of snapping
                double error = normalizeAngle(headingHoldRad - hwManager.getHeading());
                if (Math.abs(error) >= HOLD_TOLERANCE) {
                    double correction = HOLD_KP * error;
                    if (Math.abs(correction) > HOLD_MAX_POWER) {
                        correction = Math.signum(correction) * HOLD_MAX_POWER;
                    }
                    driveRx = correction;
                }
            }
        }

        drivetrain.drive(x * speedMult, y * speedMult, driveRx);

        if (gamepad1.options && !prevOptions) {
            drivetrain.resetYaw();
            autoRotating = false;
            headingHoldActive = false;
            headingHoldDelayTimer.reset();
        }

        if (gamepad1.share && !prevShare) {
            headingHoldEnabled = !headingHoldEnabled;
            headingHoldActive = false;
            headingHoldDelayTimer.reset();
        }
        prevShare = gamepad1.share;

        // Live tuning: gamepad 1 D-pad up/down = drive top speed, gamepad 2 D-pad up/down = intake speed
        if (gamepad1.dpad_up && !prevDPadUp)     drivetrain.adjustMaxSpeed(1);
        if (gamepad1.dpad_down && !prevDPadDown) drivetrain.adjustMaxSpeed(-1);
        if (gamepad2.dpad_up && !prevG2Up)       intake.adjustSpeed(1);
        if (gamepad2.dpad_down && !prevG2Down)   intake.adjustSpeed(-1);
        prevDPadUp   = gamepad1.dpad_up;
        prevDPadDown = gamepad1.dpad_down;
        prevG2Up     = gamepad2.dpad_up;
        prevG2Down   = gamepad2.dpad_down;

        if (gamepad1.dpad_right && !prevDPadRight) {
            currentTargetHeading = targetHeadingFarRad;
            autoRotating = true;
        }
        if (gamepad1.dpad_left && !prevDPadLeft) {
            currentTargetHeading = targetHeading45Rad;
            autoRotating = true;
        }

        handleBalls();
        telemetry.update();

        prevRBumper   = gamepad1.right_bumper;
        prevLBumper   = gamepad1.left_bumper;
        prevDPadRight = gamepad1.dpad_right;
        prevDPadLeft  = gamepad1.dpad_left;
        prevOptions   = gamepad1.options;
    }

    // Intake, feeder and shooter. Every button is tap on / tap off. Shooting wins over everything
    // else; circle (backwards) turns off loading and cross.
    private void handleBalls() {
        boolean triangle = gamepad1.triangle && !prevTriangle;
        boolean square   = gamepad1.square && !prevSquare;
        boolean cross    = gamepad1.cross && !prevCross;
        boolean circle   = gamepad1.circle && !prevCircle;
        prevTriangle = gamepad1.triangle;
        prevSquare   = gamepad1.square;
        prevCross    = gamepad1.cross;
        prevCircle   = gamepad1.circle;

        if (gamepad1.left_bumper && !prevLBumper) preSpin = !preSpin;

        int wasShooting = shootMode;
        if (triangle) shootMode = shootMode == SHOOT_CLOSE ? NOT_SHOOTING : SHOOT_CLOSE;
        if (square)   shootMode = shootMode == SHOOT_FAR   ? NOT_SHOOTING : SHOOT_FAR;

        if (shootMode != NOT_SHOOTING) {
            int target = shootMode == SHOOT_FAR ? FAR_SHOT_VELOCITY : CLOSE_SHOT_VELOCITY;
            if (wasShooting == NOT_SHOOTING) {
                feedingShot = false;
                loading = feedingUp = reversing = false;
            }
            shooter.spinTo(target);

            double vel = hwManager.shooterVelocity();
            if (!feedingShot && Math.abs(vel - target) < READY_TOLERANCE) {
                feedingShot = true;
            } else if (feedingShot && vel < target - DROP_TOLERANCE) {
                feedingShot = false; // a ball just took speed off the wheel: pause until it recovers
            }
            // nothing reaches the wheel before it is at speed
            setFeeder(feedingShot ? SHOT_FEED_VELOCITY : 0);
            setIntake(feedingShot ? 1 : 0);
            telemetry.addData("Shooter", "%.0f / %d  %s  (tap again to stop)", vel, target,
                    feedingShot ? "FEEDING" : "spinning up");
            return;
        }

        if (wasShooting != NOT_SHOOTING) feedingShot = false;
        if (preSpin) shooter.spinTo(CLOSE_SHOT_VELOCITY);
        else shooter.turnOff();

        if (gamepad1.right_bumper && !prevRBumper) {
            loading = !loading;
            if (loading) reversing = false;
        }
        if (cross) {
            feedingUp = !feedingUp;
            if (feedingUp) reversing = false;
        }
        if (circle) {
            reversing = !reversing;
            if (reversing) loading = feedingUp = false;
        }

        if (reversing) {
            setIntake(-1);
            setFeeder(-REVERSE_VELOCITY);
        } else {
            setIntake(loading ? 1 : 0);
            if (feedingUp)    setFeeder(FEED_UP_VELOCITY);
            else if (loading) setFeeder(buttonPressed ? 0 : supply.LOAD_VELOCITY); // stops at the button sensor, if fitted
            else              setFeeder(0);
        }

        telemetry.addData("Shooter", preSpin ? String.format("pre-spinning %.0f", hwManager.shooterVelocity()) : "off (L1 to pre-spin)");
        telemetry.addData("Balls", reversing ? "BACKWARDS" : feedingUp ? "feeder pushing up" : loading ? "loading" : "stopped");
    }

    private void setFeeder(int velocity) {
        if (velocity == lastFeeder) return;
        lastFeeder = velocity;
        supply.turnOff();
        if (velocity != 0) supply.turnOn(velocity);
    }

    private void setIntake(int direction) {
        if (direction == lastIntake) return;
        lastIntake = direction;
        if (direction > 0)      intake.turnOn();
        else if (direction < 0) intake.reverseOn();
        else                    intake.stop();
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // G404: everything unpowered once TELEOP is over
    private void stopAll() {
        drivetrain.stop();
        shooter.turnOff();
        supply.turnOff();
        intake.stop();
        loading = preSpin = feedingShot = feedingUp = reversing = false;
        shootMode = NOT_SHOOTING;
        lastFeeder = lastIntake = 0;
        autoRotating      = false;
        headingHoldActive = false;
    }
}