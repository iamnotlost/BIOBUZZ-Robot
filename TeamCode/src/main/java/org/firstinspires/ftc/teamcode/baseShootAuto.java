package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Shooting AUTO: from the START spot, drive to the saved SHOOT spot (or shoot right from the start if the shoot
// spot was saved there), shoot all 4 pre-loaded POLLEN into the upward CELL (it already holds 3 NECTAR, so 4
// POLLEN can TIP the HIVE: 20 pts), then drive to the saved END spot. Spots are taught with setupAutoRed /
// setupAutoBlue, as PLAN A or PLAN B (two different start spots, chosen at INIT with the D-pad).
//
// Game rules this follows (2026-27 Competition Manual):
//   G304  start touching the wall, on our side, NOT in the LOADING ZONE, with 4 POLLEN, motionless after INIT
//         (nothing moves in INIT here; the D-pad only picks the plan)
//   10.5  LEAVE (3) = not touching the wall and PARK (5) = partly in our LOADING ZONE, both judged at the END
//         of AUTO, so the END spot should be in the LOADING ZONE without touching the wall. Driving back to
//         the start spot would score neither.
//   G402  stay on our own side (the taught spots decide this)
//   G403  everything is stopped SAFETY_MARGIN_S before 0:30, so nothing moves in the transition
//   G417  only LAUNCH into the upward CELL, never touch the HIVE
public abstract class baseShootAuto extends baseAuto {
    protected abstract Alliance alliance();

    public static int    PRELOADS       = 4;
    public static double DRIVE_MAX_S    = 8;     // give up on a drive after this long (e.g. blocked)
    public static double SHOOT_MAX_S    = 8;     // give up shooting after this long
    public static double EMPTY_S        = 1.5;   // feeding this long without a ball = robot is empty
    public static double LAST_BALL_S    = 0.3;   // keep feeding this long after the 4th ball so it leaves the wheel
    public static double END_RESERVE_S  = 5;     // stop shooting with this much AUTO left, to reach the END spot

    burst burst;

    @Override
    public void runOpMode() {
        setup();
        burst = new burst(hwManager, shooter, supply, intake);
        String setupName = "setupAuto" + (alliance() == Alliance.RED ? "Red" : "Blue");

        // INIT: pick the plan with the D-pad, check everything, keep the robot still
        boolean prevLeft = false, prevRight = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_left && !prevLeft)   autoSpots.plan = 0;
            if (gamepad1.dpad_right && !prevRight) autoSpots.plan = 1;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            int p = autoSpots.plan;
            follower.update();

            telemetry.addData("Auto", "SHOOT %s   PLAN %s   (D-pad left = A, right = B)", alliance(), autoSpots.planName(p));
            telemetry.addData("Start", "plan %s START spot, touching the wall, 4 POLLEN loaded", autoSpots.planName(p));
            telemetry.addData("Shoot from", autoSpots.shootText(alliance(), p));
            telemetry.addData("End at", autoSpots.text(autoSpots.end(alliance(), p)));
            if (!autoSpots.taught(alliance(), p)) {
                telemetry.addLine("!! PLAN " + autoSpots.planName(p) + " NOT SET UP - run " + setupName
                        + " (it would only shoot from the start and drive straight off the wall)");
            }
            for (double[] s : new double[][] {autoSpots.shoot(alliance(), p), autoSpots.end(alliance(), p)}) {
                if (Math.hypot(s[0], s[1]) > MAX_SPOT_IN) {
                    telemetry.addLine(String.format("!! a spot is %.0f ft from START - taught wrong, run %s again (it won't drive there)",
                            Math.hypot(s[0], s[1]) / 12, setupName));
                }
            }
            telemetry.addData("Shot speed", "%d", baseTeleOp.SHOT_VELOCITY);
            double volts = hwManager.batteryVoltage();
            telemetry.addData("Battery", "%.1f V%s", volts, volts < 12.5 ? "  !! LOW - charge or swap" : "");
            if (!hwManager.missing.isEmpty()) telemetry.addData("!! Not found", hwManager.missingText());
            telemetry.addData("Odometry", "%s  (should stay 0, 0, 0 while still)", poseText(follower.getPose()));
            telemetry.update();
        }
        if (isStopRequested()) return;

        int p = autoSpots.plan;
        double[] shootSpot = autoSpots.shoot(alliance(), p);
        double[] endSpot = autoSpots.end(alliance(), p);
        boolean taught = autoSpots.taught(alliance(), p);
        startTimer();
        event(String.format("PLAN %s, shot speed %d, battery %.1f V%s", autoSpots.planName(p),
                baseTeleOp.SHOT_VELOCITY, hwManager.batteryVoltage(), taught ? "" : ", PLAN NOT SET UP"));

        waitSeconds(baseParkAuto.START_DELAY_S);
        if (autoSpots.isSet(shootSpot)) {
            step("1/3 drive to SHOOT spot");
            goTo("SHOOT spot", shootSpot, DRIVE_MAX_S);
        } else {
            event("shooting from the START spot (no drive)");
        }
        step("2/3 shoot " + PRELOADS + " balls");
        shootAll();
        if (taught) {
            step("3/3 drive to END spot");
            goTo("END spot", endSpot, DRIVE_MAX_S);
        } else {
            // nothing taught: at least get off the wall for LEAVE
            step("3/3 drive off the wall (plan not set up)");
            runPath(lineTo(baseParkAuto.LEAVE_DISTANCE, 0), DRIVE_MAX_S);
        }
        finish();
    }

    private void shootAll() {
        burst.start();
        ElapsedTime shootTimer = new ElapsedTime();
        ElapsedTime afterLast = null;
        boolean wasStarted = false;
        int lastBalls = 0;
        String why = null;
        while (true) {
            if (!running())                          { why = "AUTO time up"; break; }
            if (timeLeft() <= END_RESERVE_S)         { why = String.format("only %.0f s of AUTO left, going to END", END_RESERVE_S); break; }
            if (shootTimer.seconds() >= SHOOT_MAX_S) { why = String.format("gave up after %.0f s", SHOOT_MAX_S); break; }

            follower.update();
            burst.update();
            double speed = hwManager.shooterVelocity();

            if (burst.started() && !wasStarted) {
                event(String.format("wheel ready at %.0f after %.1f s, feeding", speed, shootTimer.seconds()));
            }
            wasStarted = burst.started();
            if (burst.balls() > lastBalls) {
                event(String.format("ball %d hit the wheel (wheel dropped to %.0f)", burst.balls(), speed));
                lastBalls = burst.balls();
            }

            if (burst.balls() >= PRELOADS) {
                if (afterLast == null) afterLast = new ElapsedTime();
                else if (afterLast.seconds() > LAST_BALL_S) { why = "all " + PRELOADS + " balls shot"; break; }
            }
            if (burst.started() && burst.secondsSinceLastBall() > EMPTY_S) {
                why = String.format("no ball for %.1f s - empty or stuck", EMPTY_S);
                break;
            }

            telemetry.addData("Shooter", "%s  %.0f / %d   power %.2f   balls %d of %d",
                    !burst.started() ? "spinning up" : burst.feeding() ? "FEEDING" : "waiting for wheel",
                    speed, baseTeleOp.SHOT_VELOCITY, shooter.power(), burst.balls(), PRELOADS);
            showProgress();
        }
        burst.stop();
        if (!burst.started()) why += String.format(" - wheel NEVER got ready (last %.0f of %d)",
                hwManager.shooterVelocity(), baseTeleOp.SHOT_VELOCITY);
        event(String.format("shooting stopped after %.1f s, %d balls: %s", shootTimer.seconds(), burst.balls(), why));
    }
}
