package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Shared TeleOp for both alliances; redTeleOp / blueTeleOp only pick the Alliance
public abstract class baseTeleOp extends OpMode {
    hwManager hwManager;
    drivetrain drivetrain;
    supply supply;
    shooter shooter;
    intake intake;
    matchTimer matchTimer = new matchTimer();

    protected abstract Alliance alliance();

    boolean prevCross, prevClose, prevFar;
    boolean prevRBumper, prevLBumper;
    boolean prevDPadRight, prevDPadLeft;
    boolean prevOptions, prevTriangle, prevCircle, prevSquare;

    ElapsedTime sendTimer = new ElapsedTime();
    boolean sendingAll = false;
    int sendStage = 0;
    int cstSpeed = 2000;
    boolean buttonPressed = false;
    
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

    private boolean headingHoldActive = false;
    private double  headingHoldRad    = 0;
    private final ElapsedTime headingHoldDelayTimer = new ElapsedTime();
    private static final long HEADING_HOLD_DELAY_MS = 250;

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
    }

    @Override
    public void start() {
        matchTimer.start();
    }

    @Override
    public void loop() {
        hwManager.pinpoint.update();
        matchTimer.update(gamepad1, gamepad2);

        if (matchTimer.isOver()) {
            stopAll();
            telemetry.addData("Match", "OVER - press STOP");
            telemetry.update();
            return;
        }

        shooter.update();
        buttonPressed = !hwManager.button.getState();

        telemetry.addData("Time left", matchTimer.display());
        telemetry.addData("NECTAR -> FLOWER", matchTimer.nectarFlowerAllowed() ? "ALLOWED" : "not yet");
        telemetry.addData("Button Pressed", buttonPressed);
        telemetry.addData("Shooter velocity", hwManager.shooter.getVelocity());
        telemetry.addData("Shooter follower power", hwManager.secondShooter.getPower());
        telemetry.addData("Heading (deg)", Math.toDegrees(hwManager.pinpoint.getHeading()));
        telemetry.addData("Auto-rotate", autoRotating ? "ON (target " + Math.toDegrees(currentTargetHeading) + " deg)" : "off");
        telemetry.addData("Heading hold", headingHoldActive ? "ON (target " + Math.toDegrees(headingHoldRad) + " deg)" : "off");
        telemetry.update();

        double x  = -gamepad1.left_stick_x;
        double y  =  gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        boolean slowMode  = gamepad1.right_trigger > 0.05;
        boolean superSlow = gamepad1.left_trigger  > 0.05;

        double speedMult = superSlow ? 0.25 : slowMode ? 0.5 : 1.0;

        if (autoRotating && Math.abs(gamepad1.right_stick_x) > CANCEL_STICK_THRESH) {
            autoRotating = false;
        }

        double driveRx = rx * speedMult;
        if (autoRotating) {
            double error = normalizeAngle(currentTargetHeading - hwManager.pinpoint.getHeading());
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
            if (!headingHoldActive && headingHoldDelayTimer.milliseconds() >= HEADING_HOLD_DELAY_MS) {
                headingHoldRad    = hwManager.pinpoint.getHeading();
                headingHoldActive = true;
            }
            if (headingHoldActive) {
                double error = normalizeAngle(headingHoldRad - hwManager.pinpoint.getHeading());
                if (Math.abs(error) >= HEADING_TOLERANCE) {
                    double correction = Constants.followerConstants.coefficientsHeadingPIDF.P * error
                            + Constants.followerConstants.coefficientsHeadingPIDF.F * Math.signum(error);
                    if (Math.abs(correction) > MAX_ROT_POWER) {
                        correction = Math.signum(correction) * MAX_ROT_POWER;
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

        boolean crossPressed     = gamepad1.cross;
//        boolean closePressed     = gamepad1.dpad_up;
//        boolean farPressed       = gamepad1.dpad_down;
        boolean rBumperPressed   = gamepad1.right_bumper;
        boolean lBumperPressed   = gamepad1.left_bumper;
        boolean dpadRightPressed = gamepad1.dpad_right;
        boolean dpadLeftPressed  = gamepad1.dpad_left;
        boolean trianglePressed  = gamepad1.triangle;
        boolean circlePressed    = gamepad1.circle;
        boolean squarePressed    = gamepad1.square;

        if (dpadRightPressed && !prevDPadRight) {
            currentTargetHeading = targetHeadingFarRad;
            autoRotating = true;
        }
        if (dpadLeftPressed && !prevDPadLeft) {
            currentTargetHeading = targetHeading45Rad;
            autoRotating = true;
        }

        if (sendingAll) {
            if ((trianglePressed && !prevTriangle) || (squarePressed && !prevSquare)) {
                stopSendAll();
            } else {
                handleSendAll();
            }
        } else {
            if (trianglePressed && !prevTriangle) startSendAll();
            if (squarePressed   && !prevSquare)   startSendAllFar();

            if (crossPressed   && !prevCross)    supply.toggle();
//            if (closePressed   && !prevClose)    shooter.shootClose();
//            if (farPressed     && !prevFar)      shooter.shootFar();
            if (lBumperPressed && !prevLBumper)  shooter.shootCustom(cstSpeed);
            if (circlePressed  && !prevCircle)   intake.toggleReverse();

            if (rBumperPressed && !prevRBumper) {
                intake.toggle();
                if (!buttonPressed && intake.running) supply.turnOnSlow();
            }
        }

        supply.stopOnButton(buttonPressed && shooter.currentMode == 0);

        prevCross        = crossPressed;
//        prevClose        = closePressed;
//        prevFar          = farPressed;
        prevRBumper      = rBumperPressed;
        prevLBumper      = lBumperPressed;
        prevDPadRight    = dpadRightPressed;
        prevDPadLeft     = dpadLeftPressed;
        prevOptions      = gamepad1.options;
        prevTriangle     = trianglePressed;
        prevCircle       = circlePressed;
        prevSquare       = squarePressed;
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    private static final double VELOCITY_THRESHOLD   = 200;
    private static final double RECOVERY_THRESHOLD   = 100;
    private static final double DROP_THRESHOLD       = 150;
    private static final long   SPINUP_TIMEOUT_MS    = 2500;
    private static final long   PUSH_MIN_MS          = 120;
    private static final long   PUSH_TIMEOUT_MS      = 700;
    private static final long   RELOAD_DWELL_MS      = 200;
    private static final long   RELOAD_TIMEOUT_MS    = 800;
    private static final long   RECOVERY_TIMEOUT_MS  = 800;
    private static final long   CONTINUOUS_FEED_MS   = 2000;
    private static final int    TOTAL_BALLS          = 3;

    private static final double FAR_VELOCITY            = 1860;
    private static final double FAR_VELOCITY_TOLERANCE  = 40;
    private static final long   FAR_SPINUP_TIMEOUT_MS   = 2000;
    private static final long   FAR_FEED_MS             = 3500;
    private static final int    FAR_SUPPLY_VELOCITY     = 520;

    private int     targetVelocity     = 1320;
    private int     sendSupplyVelocity = 1200;
    private int     ballsSent          = 0;
    private boolean stagedSend         = false;
    private boolean farMode            = false;

    private void startSendAll() {
        sendingAll = true;
        sendStage  = 0;
        ballsSent  = 0;
        targetVelocity     = 1320;
        sendSupplyVelocity = 1500;
        stagedSend         = false;
        sendTimer.reset();

        shooter.shootCustom(targetVelocity);
    }

    private void startSendAllFar() {
        sendingAll = true;
        sendStage  = 0;
        ballsSent  = 0;
        farMode    = true;
        stagedSend = false;
        sendTimer.reset();

        if (shooter.currentMode != 1) {
            shooter.shootFar();
        }
    }

    private void handleSendAll() {
        if (farMode)         handleSendAllFar();
        else if (stagedSend) handleSendAllStaged();
        else                 handleSendAllContinuous();
    }

    private void handleSendAllFar() {
        double currentVel = hwManager.shooter.getVelocity();

        switch (sendStage) {
            case 0: {
                boolean atSpeed  = Math.abs(currentVel - FAR_VELOCITY) < FAR_VELOCITY_TOLERANCE;
                boolean timedOut = sendTimer.milliseconds() >= FAR_SPINUP_TIMEOUT_MS;
                if (atSpeed || timedOut) {
                    intake.turnOn();
                    supply.turnOn(FAR_SUPPLY_VELOCITY);
                    sendTimer.reset();
                    sendStage = 1;
                }
                break;
            }

            case 1: {
                if (sendTimer.milliseconds() >= FAR_FEED_MS) {
                    stopSendAll();
                }
                break;
            }
        }
    }

    private void handleSendAllContinuous() {
        double currentVel = hwManager.shooter.getVelocity();

        switch (sendStage) {
            case 0: {
                boolean atSpeed  = Math.abs(currentVel - targetVelocity) < VELOCITY_THRESHOLD;
                boolean timedOut = sendTimer.milliseconds() >= SPINUP_TIMEOUT_MS;
                if (atSpeed || timedOut) {
                    supply.turnOn(sendSupplyVelocity);
                    intake.turnOn();
                    sendTimer.reset();
                    sendStage = 1;
                }
                break;
            }

            case 1: {
                if (sendTimer.milliseconds() >= CONTINUOUS_FEED_MS) {
                    stopSendAll();
                }
                break;
            }
        }
    }

    private void handleSendAllStaged() {
        double currentVel = hwManager.shooter.getVelocity();
        boolean btn = !hwManager.button.getState();

        switch (sendStage) {
            case 0: {
                boolean atSpeed  = Math.abs(currentVel - targetVelocity) < VELOCITY_THRESHOLD;
                boolean timedOut = sendTimer.milliseconds() >= SPINUP_TIMEOUT_MS;
                if (atSpeed || timedOut) {
                    supply.turnOn(sendSupplyVelocity);
                    sendTimer.reset();
                    sendStage = 1;
                }
                break;
            }

            case 1: {
                boolean ballLeft = sendTimer.milliseconds() >= PUSH_MIN_MS
                        && currentVel < targetVelocity - DROP_THRESHOLD;
                boolean timedOut = sendTimer.milliseconds() >= PUSH_TIMEOUT_MS;
                if (ballLeft || timedOut) {
                    supply.turnOff();
                    ballsSent++;
                    if (ballsSent >= TOTAL_BALLS) {
                        stopSendAll();
                    } else {
                        sendTimer.reset();
                        sendStage = 2;
                    }
                }
                break;
            }

            case 2: {
                intake.turnOn();
                supply.turnOn(sendSupplyVelocity);
                sendTimer.reset();
                sendStage = 3;
                break;
            }

            case 3: {
                boolean dwellOk  = sendTimer.milliseconds() >= RELOAD_DWELL_MS;
                boolean seated   = btn && dwellOk;
                boolean timedOut = sendTimer.milliseconds() >= RELOAD_TIMEOUT_MS;
                if (seated || timedOut) {
                    intake.turnOff();
                    supply.turnOff();
                    sendTimer.reset();
                    sendStage = 4;
                }
                break;
            }

            case 4: {
                boolean recovered = Math.abs(currentVel - targetVelocity) < RECOVERY_THRESHOLD;
                boolean timedOut  = sendTimer.milliseconds() >= RECOVERY_TIMEOUT_MS;
                if (recovered || timedOut) {
                    supply.turnOn(sendSupplyVelocity);
                    sendTimer.reset();
                    sendStage = 1;
                }
                break;
            }
        }
    }

    public void stopSendAll() {
        shooter.turnOff();
        supply.turnOff();
        intake.turnOff();
        sendingAll = false;
        sendStage  = 0;
        ballsSent  = 0;
        farMode    = false;
    }

    // G404: everything unpowered once TELEOP is over
    private void stopAll() {
        drivetrain.drive(0, 0, 0);
        stopSendAll();
        intake.stop();
        autoRotating      = false;
        headingHoldActive = false;
    }
}