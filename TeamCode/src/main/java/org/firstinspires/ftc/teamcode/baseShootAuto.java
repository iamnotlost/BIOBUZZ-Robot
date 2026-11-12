package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Shooting AUTO: from the START spot, drive to the saved SHOOT spot, shoot all 4 pre-loaded POLLEN into
// the upward CELL (it already holds 3 NECTAR, so 4 POLLEN can TIP the HIVE: 20 pts), then drive to the
// saved END spot. Spots are taught with setupAutoRed / setupAutoBlue.
//
// Game rules this follows (2026-27 Competition Manual):
//   G304  start touching the wall, on our side, NOT in the LOADING ZONE, with 4 POLLEN, motionless after INIT
//         (nothing moves in INIT here)
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
        double[] shootSpot = autoSpots.shoot(alliance());
        double[] endSpot = autoSpots.end(alliance());

        telemetry.addData("Auto", "SHOOT " + alliance());
        telemetry.addData("Shoot spot", autoSpots.text(shootSpot));
        telemetry.addData("End spot", autoSpots.text(endSpot));
        if (!autoSpots.isSet(shootSpot) || !autoSpots.isSet(endSpot)) {
            telemetry.addLine("!! Not set up yet - run setupAuto" + (alliance() == Alliance.RED ? "Red" : "Blue") + " first");
        }
        telemetry.addData("Shot speed", baseTeleOp.SHOT_VELOCITY);
        telemetry.addLine("Robot on the START spot (tape), touching the wall, 4 POLLEN loaded. Keep it STILL.");
        telemetry.update();

        waitForStart();
        startTimer();

        waitSeconds(baseParkAuto.START_DELAY_S);
        if (autoSpots.isSet(shootSpot)) goTo(shootSpot, DRIVE_MAX_S);
        shootAll();
        if (autoSpots.isSet(endSpot)) {
            goTo(endSpot, DRIVE_MAX_S);
        } else if (!autoSpots.isSet(shootSpot)) {
            // nothing taught: at least get off the wall for LEAVE
            runPath(lineTo(baseParkAuto.LEAVE_DISTANCE, 0), DRIVE_MAX_S);
        }

        stopAll();
    }

    private void shootAll() {
        burst.start();
        ElapsedTime shootTimer = new ElapsedTime();
        ElapsedTime afterLast = null;
        while (running() && timeLeft() > END_RESERVE_S && shootTimer.seconds() < SHOOT_MAX_S) {
            follower.update();
            burst.update();
            if (burst.balls() >= PRELOADS) {
                if (afterLast == null) afterLast = new ElapsedTime();
                else if (afterLast.seconds() > LAST_BALL_S) break;
            }
            if (burst.started() && burst.secondsSinceLastBall() > EMPTY_S) break;   // nothing left to shoot
            telemetry.addData("Shooting", "%.0f / %d   balls: %d of %d",
                    hwManager.shooterVelocity(), baseTeleOp.SHOT_VELOCITY, burst.balls(), PRELOADS);
            showProgress();
        }
        burst.stop();
    }
}
