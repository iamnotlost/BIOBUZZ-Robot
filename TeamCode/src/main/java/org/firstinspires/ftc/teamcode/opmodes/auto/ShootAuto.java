package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequencer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSpots;

/**
 * Shooting AUTO: START -> (path points) -> SHOOT spot -> shoot the 4 pre-loaded POLLEN -> (path points) -> END spot.
 * The upward CELL already holds 3 NECTAR, so 4 POLLEN can TIP the HIVE (20 pts). Spots are taught with
 * setupAutoRed / setupAutoBlue as plan A or B, picked with the D-pad at INIT.
 *
 * Game rules (2026-27 manual): G304 start touching the wall, own side, not in the LOADING ZONE, 4 POLLEN,
 * motionless after INIT. LEAVE (not touching the wall) and PARK (partly in our LOADING ZONE) are judged at the end
 * of AUTO, so the END spot belongs in the LOADING ZONE, off the wall. G402 stay on our side. G417 only launch into
 * the upward CELL.
 */
public abstract class ShootAuto extends BaseAuto {
    public static int PRELOADS = 4;
    public static double DRIVE_MAX_S = 8;      // give up on a drive after this long (blocked?)
    public static double SHOOT_MAX_S = 8;      // give up shooting after this long
    public static double EMPTY_S = 1.5;        // feeding this long without a ball = empty or stuck
    public static double LAST_BALL_S = 0.3;    // keep feeding after the last ball so it leaves the wheel
    public static double END_RESERVE_S = 5;    // stop shooting with this much AUTO left, to reach the END spot

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        setup();
        waitInInit();
        if (isStopRequested()) return;

        int plan = AutoSpots.plan;
        double[] shootSpot = AutoSpots.shoot(alliance(), plan);
        boolean taught = AutoSpots.taught(alliance(), plan);
        startTimer();
        event(String.format("PLAN %s, shot speed %d, battery %.1f V%s", AutoSpots.planName(plan),
                ShotSequencer.SHOT_VELOCITY, hw.batteryVoltage(), taught ? "" : ", PLAN NOT SET UP"));

        waitSeconds(START_DELAY_S);
        if (AutoSpots.isSet(shootSpot)) {
            step("1/3 drive to SHOOT spot");
            goTo("SHOOT spot", shootSpot, AutoSpots.points(AutoSpots.route(alliance(), plan, true)), DRIVE_MAX_S);
        } else {
            event("shooting from the START spot (no drive)");
        }
        step("2/3 shoot " + PRELOADS + " balls");
        shootAll();
        if (taught) {
            step("3/3 drive to END spot");
            goTo("END spot", AutoSpots.end(alliance(), plan),
                    AutoSpots.points(AutoSpots.route(alliance(), plan, false)), DRIVE_MAX_S);
        } else {
            step("3/3 drive off the wall (plan not set up)");   // at least LEAVE
            goTo("off the wall", new double[] {LEAVE_DISTANCE, 0, 0}, DRIVE_MAX_S);
        }
        finish();
    }

    // INIT: pick the plan with the D-pad, show what will happen and the checks. Nothing moves.
    private void waitInInit() {
        String setupName = "setupAuto" + alliance().title();
        while (opModeInInit()) {
            if (gamepad1.dpadLeftWasPressed()) AutoSpots.plan = 0;
            if (gamepad1.dpadRightWasPressed()) AutoSpots.plan = 1;
            int plan = AutoSpots.plan;
            follower.update();

            telemetry.addData("Auto", "SHOOT %s   PLAN %s   (D-pad left = A, right = B)", alliance(), AutoSpots.planName(plan));
            telemetry.addData("Start", "plan %s START spot, touching the wall, 4 POLLEN loaded", AutoSpots.planName(plan));
            telemetry.addData("Shoot from", AutoSpots.shootText(alliance(), plan));
            telemetry.addData("End at", AutoSpots.text(AutoSpots.end(alliance(), plan)));
            telemetry.addData("Path points", "%d on the way to shoot, %d on the way to the LOADING ZONE",
                    AutoSpots.points(AutoSpots.route(alliance(), plan, true)).size(),
                    AutoSpots.points(AutoSpots.route(alliance(), plan, false)).size());
            if (!AutoSpots.taught(alliance(), plan)) {
                telemetry.addLine("!! PLAN " + AutoSpots.planName(plan) + " NOT SET UP - run " + setupName
                        + " (it would only shoot from the start and drive straight off the wall)");
            }
            for (double[] s : new double[][] {AutoSpots.shoot(alliance(), plan), AutoSpots.end(alliance(), plan)}) {
                double far = Math.hypot(s[0], s[1]);
                if (far > MAX_SPOT_IN) {
                    telemetry.addLine(String.format("!! a spot is %.0f ft from START - taught wrong, run %s again", far / 12, setupName));
                }
            }
            telemetry.addData("Shot speed", "%d", ShotSequencer.SHOT_VELOCITY);
            showInitChecks();
            telemetry.update();
        }
    }

    private void shootAll() {
        shots.start();
        ElapsedTime shootTimer = new ElapsedTime();
        ElapsedTime afterLast = null;
        String why;
        while (true) {
            if (!running())                          { why = "AUTO time up"; break; }
            if (timeLeft() <= END_RESERVE_S)         { why = String.format("only %.0f s of AUTO left, going to END", END_RESERVE_S); break; }
            if (shootTimer.seconds() >= SHOOT_MAX_S) { why = String.format("gave up after %.0f s", SHOOT_MAX_S); break; }

            hw.update(RobotHardware.Odometry.NONE);   // same as tick(), with the shot step before the shooter update
            shots.update();
            follower.update();
            shooter.update();
            double speed = hw.shooterVelocity();

            if (shots.firstBallNow()) {
                event(String.format("wheel ready at %.0f after %.1f s, feeding", speed, shootTimer.seconds()));
            }
            if (shots.ballHitNow()) {
                event(String.format("ball %d hit the wheel (wheel dropped to %.0f)", shots.balls(), speed));
            }
            if (shots.balls() >= PRELOADS) {
                if (afterLast == null) afterLast = new ElapsedTime();
                else if (afterLast.seconds() > LAST_BALL_S) { why = "all " + PRELOADS + " balls shot"; break; }
            }
            if (shots.started() && shots.secondsSinceLastBall() > EMPTY_S) {
                why = String.format("no ball for %.1f s - empty or stuck", EMPTY_S);
                break;
            }

            telemetry.addData("Shooter", "%s  %.0f / %d   power %.2f   balls %d of %d",
                    !shots.started() ? "spinning up" : shots.feeding() ? "FEEDING" : "waiting for wheel",
                    speed, ShotSequencer.SHOT_VELOCITY, shooter.power(), shots.balls(), PRELOADS);
            showProgress();
        }
        boolean everReady = shots.started();
        double lastSpeed = hw.shooterVelocity();
        shots.stop();
        if (!everReady) why += String.format(" - wheel NEVER got ready (last %.0f of %d)", lastSpeed, ShotSequencer.SHOT_VELOCITY);
        event(String.format("shooting stopped after %.1f s, %d balls: %s", shootTimer.seconds(), shots.balls(), why));
    }
}
