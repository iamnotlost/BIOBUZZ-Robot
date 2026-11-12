package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


// Shared TeleOp for both alliances; redTeleOp / blueTeleOp only pick the Alliance
public abstract class baseTeleOp extends OpMode {
    hwManager hwManager;
    drivetrain drivetrain;
    supply supply;
    shooter shooter;
    intake intake;
    matchTimer matchTimer = new matchTimer();

    protected abstract Alliance alliance();

    boolean prevDPadUp, prevDPadDown, prevG2Up, prevG2Down, prevG2RBumper, prevG2LBumper;
    boolean prevDPadRight, prevDPadLeft;
    boolean prevOptions;
    // Start-up choices; changeable and saved in settingsMenu
    public static boolean FIELD_CENTRIC_DEFAULT = false;
    public static boolean HEADING_HOLD_DEFAULT  = false;
    boolean fieldCentric = false;
    boolean prevLeftStickButton, prevRightStickButton;
    // Drive top speeds the right stick click steps through
    private static final double[] DRIVE_SPEED_LEVELS = {0.4, 0.6, 0.8, 1.0};

    // Shooting: hold triangle, let go to stop. Every ball leaves at the same wheel speed, so every
    // shot flies to the same height:
    //   1. back off (optional): feeder runs backwards briefly so no ball is pressed against the wheel
    //   2. spin up: the wheel speeds up with nothing touching it
    //   3. feed: the first ball goes in ONLY once the wheel is at SHOT_VELOCITY and steady. Then the
    //      balls go one after another: after each ball the wheel control gets it back to speed (without
    //      overshooting), and with WAIT_BETWEEN_BALLS the feeder pauses just until it is within
    //      READY_TOLERANCE again (no extra wait), so every ball leaves at the same speed.
    //      There is no "feed anyway" time limit: a ball never goes in while the wheel is too slow.
    // 0 = no back-off: balls are no longer pushed against the wheel while loading, and backing off
    // could knock a ball out of place. Raise (e.g. 0.3) if a ball ever sits against the wheel.
    public static double BACK_OFF_SECONDS = 0;
    // Feeder speed while R1 loads; 0 = feeder still so balls settle in place
    public static int FEED_WHILE_LOADING = 0;
    // The one shot speed (ticks/s). D-pad up/down changes it by 20 and saves it.
    // (1240 = shot into the net every time on 2026-11-12)
    public static int SHOT_VELOCITY = 1240;
    // This far below the shot speed = a ball just hit the wheel: stop feeding until it is back at speed
    public static double DROP_TOLERANCE = 80;
    // true: after each ball the feeder pauses until the wheel is back at speed (same height, a tiny bit slower)
    // false: after the first ball the feeder never stops (fastest, later balls may fly lower). In settingsMenu.
    public static boolean WAIT_BETWEEN_BALLS = true;
    private boolean wasDip = false;
    // Waiting longer than this for the wheel = it cannot reach SHOT_VELOCITY (too high, or battery low)
    private static final double CANT_REACH_SECONDS = 2.0;
    private int ballsShot = 0;
    private String savedMessage = "";
    private final ElapsedTime savedMessageTimer = new ElapsedTime();
    boolean prevG2Left, prevG2Right, prevG2Cross;
    public static int SHOT_FEED_VELOCITY  = 1500;
    // Feeder speed for cross (push balls up) and circle (backwards)
    public static int FEED_UP_VELOCITY    = 1400;
    public static int REVERSE_VELOCITY    = 800;
    // Ready to feed = wheel within this of the target (above OR below) for SETTLE_MS, so every ball
    // leaves at the same speed instead of catching the wheel while it speeds up or overshoots
    // 50 = the version that shot into the net every time. (Tried 20 with an averaged speed on 2026-11-12:
    // the feeder kept starting and stopping, balls stuck against the wheel and shots came out weak.)
    public static double READY_TOLERANCE = 50;
    public static double SETTLE_MS = 100;

    boolean shooting = false;     // triangle
    boolean loading = false;      // R1
    boolean preSpin = false;      // L1
    boolean feedingUp = false;    // cross
    boolean reversing = false;    // circle
    boolean feedingShot = false;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final ElapsedTime shotLogTimer = new ElapsedTime();
    public static final int SHOT_SPEED_STEP = 10;   // 10: 1200 hit the rim and 1220 bounced out, 1210 is in between
    private boolean firstBall = true;
    // last speeds sent, so motors are only told when something changes
    private int lastFeeder = 0;
    private int lastIntake = 0;   // 1 in, -1 out, 0 stopped
    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean wasInWindow = false;

    // Unjam: if the intake is on but barely turning for JAM_SECONDS, run it backwards briefly
    public static double JAM_SECONDS    = 0.4;
    public static double UNJAM_SECONDS  = 0.3;
    public static double JAM_FRACTION   = 0.3;   // below this share of the set speed = stuck
    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime unjamTimer = new ElapsedTime();
    private boolean unjamming = false;
    boolean buttonPressed = false;
    boolean prevOpponentNectar = false;
    
    // Slowest speed when a trigger is squeezed all the way
    public static double MIN_TRIGGER_SPEED = 0.25;
    // D-pad left/right turn-to-angle presets: off until the angles are set for the HIVE
    public static boolean PRESET_TURNS_ENABLED = false;

    // What the driver screen shows about balls and the shooter (set in handleBalls)
    private String shooterStatus = "";
    private String ballStatus = "";
    private boolean readyRumbled = false;

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
    private boolean headingHoldEnabled = false;
    // If holding needs more than this much correction, the heading reading is wrong (e.g. the Pinpoint was
    // bumped while calibrating in INIT): switch holding off instead of turning the robot round and round
    private static final double HOLD_GIVE_UP = Math.toRadians(30);
    private boolean holdGaveUp = false;
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
        boolean loaded = settings.load();
        fieldCentric = FIELD_CENTRIC_DEFAULT;
        headingHoldEnabled = HEADING_HOLD_DEFAULT;

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
        telemetry.addData("Settings", loaded ? "loaded" : "could not read the file - using defaults");
        telemetry.addData("!!", "Keep the robot STILL during INIT (the Pinpoint calibrates)");
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

        // Without a heading (Pinpoint lost and no IMU), only robot-centric driving works
        if (!hwManager.hasHeading()) {
            fieldCentric = false;
            headingHoldEnabled = false;
            headingHoldActive = false;
            autoRotating = false;
        }

        if (matchTimer.isOver()) {
            stopAll();
            telemetry.addData("Match", "OVER - press STOP");
            telemetry.update();
            return;
        }

        buttonPressed = hwManager.buttonPressed();

        boolean opponentNectar = hwManager.elementSensor.isOpponentNectar(alliance());
        if (opponentNectar && !prevOpponentNectar) gamepad1.rumble(500);
        prevOpponentNectar = opponentNectar;

        // Squeeze either trigger to slow down gradually: the harder you squeeze, the slower (down to 25%)
        double squeeze = Math.max(gamepad1.left_trigger, gamepad1.right_trigger);
        double speedMult = 1.0 - (1.0 - MIN_TRIGGER_SPEED) * squeeze;

        // Left stick as one arrow: dead zone and stick feel act on how far it is pushed, and the direction is
        // kept exactly, so pushing straight ahead never drifts sideways. The trigger lowers the top speed only,
        // so small pushes still move the robot while slowed down.
        double sx = -gamepad1.left_stick_x, sy = gamepad1.left_stick_y;
        double push = Math.hypot(sx, sy);
        double movePower = drivetrain.shape(push, drivetrain.MAX_SPEED * speedMult, drivetrain.MIN_DRIVE_POWER);
        double x  = push > 0 ? sx / push * movePower : 0;
        double y  = push > 0 ? sy / push * movePower : 0;
        double rx = -drivetrain.shape(gamepad1.right_stick_x, drivetrain.TURN_SPEED * speedMult, drivetrain.MIN_TURN_POWER);

        if (autoRotating && Math.abs(gamepad1.right_stick_x) > CANCEL_STICK_THRESH) {
            autoRotating = false;
        }

        double driveRx = rx;
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
                if (Math.abs(error) > HOLD_GIVE_UP) {
                    headingHoldEnabled = false;
                    headingHoldActive = false;
                    holdGaveUp = true;
                } else if (Math.abs(error) >= HOLD_TOLERANCE) {
                    double correction = HOLD_KP * error;
                    if (Math.abs(correction) > HOLD_MAX_POWER) {
                        correction = Math.signum(correction) * HOLD_MAX_POWER;
                    }
                    driveRx = correction;
                }
            }
        }

        // Click the left stick to switch: robot-centric (forward = where the robot faces, the default)
        // or field-centric (forward = away from the driver, whichever way the robot faces)
        if (gamepad1.left_stick_button && !prevLeftStickButton && hwManager.hasHeading()) fieldCentric = !fieldCentric;
        prevLeftStickButton = gamepad1.left_stick_button;

        if (fieldCentric) drivetrain.drive(x, y, driveRx);
        else              drivetrain.driveRobotCentric(x, y, driveRx);

        if (gamepad1.options && !prevOptions) {
            drivetrain.resetYaw();
            autoRotating = false;
            headingHoldActive = false;
            headingHoldDelayTimer.reset();
        }

        if (gamepad1.share && !prevShare && hwManager.hasHeading()) {
            headingHoldEnabled = !headingHoldEnabled;
            headingHoldActive = false;
            holdGaveUp = false;
            headingHoldDelayTimer.reset();
        }
        prevShare = gamepad1.share;

        // Gamepad 1 D-pad up/down: shot stronger/weaker by 10, saved on the robot at once,
        // so shots can be tuned with one controller
        if (gamepad1.dpad_up && !prevDPadUp)     changeShotSpeed(SHOT_SPEED_STEP);
        if (gamepad1.dpad_down && !prevDPadDown) changeShotSpeed(-SHOT_SPEED_STEP);
        // Gamepad 2: D-pad up/down = intake speed, R1/L1 = drive top speed
        if (gamepad2.dpad_up && !prevG2Up)       intake.adjustSpeed(1);
        if (gamepad2.dpad_down && !prevG2Down)   intake.adjustSpeed(-1);
        if (gamepad2.right_bumper && !prevG2RBumper) { drivetrain.adjustMaxSpeed(1);  saveSettings(); }
        if (gamepad2.left_bumper && !prevG2LBumper)  { drivetrain.adjustMaxSpeed(-1); saveSettings(); }
        // Click the right stick: drive top speed 40 -> 60 -> 80 -> 100% -> back to 40%, remembered on the robot
        if (gamepad1.right_stick_button && !prevRightStickButton) nextDriveSpeed();
        prevRightStickButton = gamepad1.right_stick_button;
        prevDPadUp    = gamepad1.dpad_up;
        prevDPadDown  = gamepad1.dpad_down;
        prevG2Up      = gamepad2.dpad_up;
        prevG2Down    = gamepad2.dpad_down;
        prevG2RBumper = gamepad2.right_bumper;
        prevG2LBumper = gamepad2.left_bumper;

        if (PRESET_TURNS_ENABLED && gamepad1.dpad_right && !prevDPadRight) {
            currentTargetHeading = targetHeadingFarRad;
            autoRotating = true;
        }
        if (PRESET_TURNS_ENABLED && gamepad1.dpad_left && !prevDPadLeft) {
            currentTargetHeading = targetHeading45Rad;
            autoRotating = true;
        }

        // Gamepad 2 at the field: D-pad left/right = shot speed -/+ 10 (saved at once), cross = SAVE all settings
        int g2Step = (gamepad2.dpad_right && !prevG2Right ? SHOT_SPEED_STEP : 0)
                   - (gamepad2.dpad_left && !prevG2Left ? SHOT_SPEED_STEP : 0);
        if (g2Step != 0) changeShotSpeed(g2Step);
        if (gamepad2.cross && !prevG2Cross) gamepad2.rumble(saveSettings() ? 200 : 800);
        prevG2Left  = gamepad2.dpad_left;
        prevG2Right = gamepad2.dpad_right;
        prevG2Cross = gamepad2.cross;

        handleBalls();
        shooter.update();   // after handleBalls, so the wheel gets this loop's target at once
        showDriverScreen();

        prevDPadRight = gamepad1.dpad_right;
        prevDPadLeft  = gamepad1.dpad_left;
        prevOptions   = gamepad1.options;
    }

    // Intake, feeder and shooter. Every ball button works while it is HELD and stops when you let go.
    // Shooting wins over everything else; circle (backwards) wins over R1 and cross.
    private void handleBalls() {
        preSpin   = gamepad1.left_bumper;
        reversing = gamepad1.circle;
        loading   = gamepad1.right_bumper && !reversing;
        feedingUp = gamepad1.cross && !reversing;

        boolean wasShooting = shooting;
        shooting = gamepad1.triangle;

        if (shooting) {
            int target = SHOT_VELOCITY;
            if (!wasShooting) {
                // a fresh start every time: wait for the wheel before the first ball goes in
                feedingShot = false;
                firstBall = true;
                wasInWindow = false;
                wasDip = true;
                ballsShot = 0;
                shotTimer.reset();
                waitTimer.reset();
            }
            shooter.spinTo(target);
            ballStatus = "going into the shooter";

            double vel = hwManager.shooterVelocity();
            if (shotTimer.seconds() < BACK_OFF_SECONDS) {
                // step 1: pull any ball off the wheel so it can spin up freely
                setFeeder(-REVERSE_VELOCITY);
                setIntake(0);
                waitTimer.reset();
                shooterStatus = "backing balls off";
                return;
            }
            // step 2: the wheel must be at speed (within READY_TOLERANCE) and stay there SETTLE_MS
            boolean inWindow = Math.abs(vel - target) <= READY_TOLERANCE;
            if (inWindow && !wasInWindow) settleTimer.reset();
            wasInWindow = inWindow;
            boolean ready = inWindow && settleTimer.milliseconds() >= SETTLE_MS;
            // a ball hitting the wheel = a sudden drop below the shot speed
            boolean dip = vel < target - DROP_TOLERANCE;
            if (dip && !wasDip && !firstBall) ballsShot++;
            wasDip = dip;
            if (!feedingShot) {
                // first ball: wheel at speed and steady. Next balls: as soon as it is back up to speed
                boolean go = firstBall ? ready : inWindow;
                if (go) {
                    if (firstBall) gamepad1.rumble(200); // short buzz: the first ball is going in
                    feedingShot = true;
                    firstBall = false;
                }
            } else if (WAIT_BETWEEN_BALLS && dip) {
                // step 3 pause: a ball just hit the wheel and slowed it. Stop feeding at once so the
                // next ball waits the moment it takes the wheel to get back to speed.
                feedingShot = false;
                waitTimer.reset();
            }
            shooter.setLearning(firstBall);  // learn the holding power only before the burst starts
            setFeeder(feedingShot ? SHOT_FEED_VELOCITY : 0);
            setIntake(feedingShot ? 1 : 0);
            logShot(target, vel);
            if (feedingShot) {
                shooterStatus = String.format("SHOOTING  %.0f / %d  (balls: %d)", vel, target, ballsShot);
            } else if (waitTimer.seconds() > CANT_REACH_SECONDS) {
                shooterStatus = String.format("wheel CAN'T REACH %d (at %.0f) - D-pad down or new battery", target, vel);
            } else {
                shooterStatus = String.format("waiting for wheel  %.0f / %d  (balls: %d)", vel, target, ballsShot);
            }
            return;
        }

        if (wasShooting) feedingShot = false;
        shooter.setLearning(true);
        if (preSpin) shooter.spinTo(SHOT_VELOCITY);
        else shooter.turnOff();

        if (reversing) {
            setIntake(-1);
            setFeeder(-REVERSE_VELOCITY);
        } else {
            if (loading) {
                if (unjamming) {
                    setIntake(-1);
                    if (unjamTimer.seconds() >= UNJAM_SECONDS) {
                        unjamming = false;
                        jamTimer.reset();
                    }
                } else {
                    setIntake(1);
                    // stuck = on but turning far slower than set; it gets JAM_SECONDS to spin up first
                    boolean slow = Math.abs(hwManager.intakeVelocity()) < intake.IN_VELOCITY * JAM_FRACTION;
                    if (!slow) jamTimer.reset();
                    else if (jamTimer.seconds() >= JAM_SECONDS) {
                        unjamming = true;
                        unjamTimer.reset();
                    }
                }
            } else {
                setIntake(0);
                unjamming = false;
                jamTimer.reset();
            }
            if (feedingUp)    setFeeder(FEED_UP_VELOCITY);
            // While loading the feeder stays still, so a ball can roll in and settle where the feeder grips it.
            // A spinning feeder knocks incoming balls out of place.
            else if (loading) setFeeder(FEED_WHILE_LOADING);
            else              setFeeder(0);
        }

        if (preSpin) {
            double vel = hwManager.shooterVelocity();
            boolean ready = Math.abs(vel - SHOT_VELOCITY) <= READY_TOLERANCE;
            if (ready && !readyRumbled) gamepad1.rumble(200); // short buzz: wheel is up to speed
            readyRumbled = ready || readyRumbled;
            shooterStatus = ready ? "READY - hold triangle" : String.format("pre-spinning %.0f / %d", vel, SHOT_VELOCITY);
        } else {
            readyRumbled = false;
            shooterStatus = "off";
        }
        ballStatus = reversing ? "BACKWARDS" : feedingUp ? "pushing up"
                : loading ? (unjamming ? "unjamming..." : "loading") : "stopped (hold R1 to load)";
    }

    // Shot recorder: while shooting, writes one line about every 50 ms to the robot log (tag SHOT) so a
    // test session can be read back afterwards. Columns: seconds since start, target, wheel speed,
    // feeding (1/0), battery volts, shooter motor power.
    private void logShot(int target, double vel) {
        if (shotLogTimer.milliseconds() < 50) return;
        shotLogTimer.reset();
        RobotLog.ii("SHOT", "%.2f,%d,%.0f,%d,%.2f,%.2f", shotTimer.seconds(), target, vel,
                feedingShot ? 1 : 0, hwManager.batteryVoltage(), shooter.power());
    }

    // Only what a driver needs during a match
    private void showDriverScreen() {
        String time = matchTimer.display();
        if (matchTimer.parkTime())                time += "   GO PARK in the LOADING ZONE";
        else if (matchTimer.nectarFlowerAllowed()) time += "   NECTAR into FLOWERS allowed";
        telemetry.addData("Time", time);
        telemetry.addData("Shooter", shooterStatus);
        telemetry.addData("Balls", ballStatus);
        telemetry.addData("Shot speed", "%d  (D-pad up/down: stronger/weaker) %s", SHOT_VELOCITY,
                savedMessageTimer.seconds() < 3 && !savedMessage.isEmpty() ? "- " + savedMessage : "");
        telemetry.addData("Intake speed", "%d  (gamepad 2 D-pad up/down)", intake.IN_VELOCITY);
        telemetry.addData("Battery", "%.1f V", hwManager.batteryVoltage());
        if (hwManager.pinpointFailed) telemetry.addData("!!", "Pinpoint lost - check its cable. Robot-centric only until restart");
        if (prevOpponentNectar) telemetry.addData("!!", "OPPONENT NECTAR - hold circle to spit it out");
        telemetry.addData("Driving", "%s, top speed %.0f%%  (L-stick click: mode, R-stick click: speed)",
                fieldCentric ? "FIELD-centric" : "ROBOT-centric", drivetrain.MAX_SPEED * 100);
        telemetry.addData("Heading hold", headingHoldEnabled ? "on (Share)" : holdGaveUp
                ? "OFF - it had to turn too far, heading reading looks wrong (Share to retry)" : "off (Share)");
        if (!hwManager.missing.isEmpty()) telemetry.addData("Not found", hwManager.missingText());
        telemetry.update();
    }

    // Shot faster (+) or slower (-), then saved so it is kept after a restart
    private void changeShotSpeed(int step) {
        SHOT_VELOCITY = Math.max(200, Math.min(2800, SHOT_VELOCITY + step));
        if (!saveSettings()) gamepad1.rumble(800); // long buzz: could not save
    }

    // Next faster level, or back to the slowest after 100%. Buzzes 1-4 times = level 1-4.
    private void nextDriveSpeed() {
        int next = 0;
        for (int i = 0; i < DRIVE_SPEED_LEVELS.length; i++) {
            if (DRIVE_SPEED_LEVELS[i] > drivetrain.MAX_SPEED + 0.01) { next = i; break; }
        }
        drivetrain.MAX_SPEED = DRIVE_SPEED_LEVELS[next];
        gamepad1.rumbleBlips(next + 1);
        saveSettings();
    }

    private boolean saveSettings() {
        boolean ok = settings.save();
        savedMessage = ok ? "SAVED" : "SAVE FAILED";
        savedMessageTimer.reset();
        return ok;
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
        loading = preSpin = feedingShot = feedingUp = reversing = shooting = false;
        lastFeeder = lastIntake = 0;
        autoRotating      = false;
        headingHoldActive = false;
    }
}