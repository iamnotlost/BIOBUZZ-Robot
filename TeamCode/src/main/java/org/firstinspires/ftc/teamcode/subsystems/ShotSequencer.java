package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Shoots balls one after another so every ball leaves at the same wheel speed (TeleOp triangle, the autos and
 * setupAuto all use this one class):
 * <ol>
 *   <li>spin up with nothing touching the wheel;</li>
 *   <li>the first ball goes in only once the wheel is within READY_TOLERANCE of SHOT_VELOCITY for SETTLE_MS;</li>
 *   <li>a ball hitting the wheel drops its speed by more than DROP_TOLERANCE: the feeder stops at once
 *       (WAIT_BETWEEN_BALLS) and the next ball goes as soon as the wheel is back within READY_TOLERANCE.</li>
 * </ol>
 * No "feed anyway" time limit: a ball never goes in while the wheel is too slow.
 * Call {@link #start()}, then {@link #update()} every loop, then {@link #stop()}.
 * While shooting it writes a SHOT line to the robot log every 50 ms:
 * seconds, target, wheel speed, feeding (1/0), battery volts, shooter power.
 */
public class ShotSequencer {
    // Saved in Settings. 1240 shot into the net every time on 2026-11-12; D-pad up/down tunes it by 5.
    public static int SHOT_VELOCITY = 1240;
    // 50: a ±20 window was tried and made the feeder start and stop, balls stuck and shots came out weak
    public static double READY_TOLERANCE = 50;
    public static double SETTLE_MS = 100;
    public static double DROP_TOLERANCE = 80;
    public static boolean WAIT_BETWEEN_BALLS = true;   // false = feeder never stops after the first ball
    public static int FEED_VELOCITY = 1500;
    public static double CANT_REACH_SECONDS = 2.0;     // waiting longer = the wheel can't reach the speed

    private final RobotHardware hw;
    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;

    private boolean active, feeding, firstBall, wasInWindow, wasDip;
    private boolean firstBallNow, ballHitNow;
    private int balls;
    private final ElapsedTime shotTimer = new ElapsedTime(), settleTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime(), sinceBall = new ElapsedTime(), logTimer = new ElapsedTime();

    public ShotSequencer(RobotHardware hw, Shooter shooter, Feeder feeder, Intake intake) {
        this.hw = hw;
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
    }

    public void start() {
        active = true;
        feeding = false;
        firstBall = true;
        wasInWindow = false;
        wasDip = true;
        balls = 0;
        shotTimer.reset();
        waitTimer.reset();
        sinceBall.reset();
    }

    public void update() {
        firstBallNow = false;
        ballHitNow = false;
        int target = SHOT_VELOCITY;
        shooter.spinTo(target);
        double vel = hw.shooterVelocity();

        boolean inWindow = Math.abs(vel - target) <= READY_TOLERANCE;
        if (inWindow && !wasInWindow) settleTimer.reset();
        wasInWindow = inWindow;
        boolean ready = inWindow && settleTimer.milliseconds() >= SETTLE_MS;

        // a ball hitting the wheel = a sudden drop below the shot speed
        boolean dip = vel < target - DROP_TOLERANCE;
        if (dip && !wasDip && !firstBall) {
            balls++;
            ballHitNow = true;
            sinceBall.reset();
        }
        wasDip = dip;

        if (!feeding) {
            if (firstBall ? ready : inWindow) {
                if (firstBall) {
                    firstBallNow = true;
                    sinceBall.reset();
                }
                feeding = true;
                firstBall = false;
            }
        } else if (WAIT_BETWEEN_BALLS && dip) {
            feeding = false;      // wait the moment it takes the wheel to get back to speed
            waitTimer.reset();
        }

        shooter.setLearning(!feeding);
        feeder.run(feeding ? FEED_VELOCITY : 0);
        if (feeding) intake.in();
        else intake.stop();
        log(target, vel);
    }

    /** Stops shooting and turns the wheel off. */
    public void stop() {
        end();
        shooter.turnOff();
    }

    /** Stops feeding but leaves the wheel to the caller (TeleOp keeps it spinning when L1 pre-spin is held). */
    public void end() {
        active = false;
        feeding = false;
        shooter.setLearning(true);
        feeder.stop();
        intake.stop();
    }

    public boolean active()               { return active; }
    public boolean feeding()              { return feeding; }
    /** The first ball has gone in. */
    public boolean started()              { return active && !firstBall; }
    /** True only in the loop the first ball started going in. */
    public boolean firstBallNow()         { return firstBallNow; }
    /** True only in the loop a ball hit the wheel. */
    public boolean ballHitNow()           { return ballHitNow; }
    public int balls()                    { return balls; }
    public double secondsSinceLastBall()  { return sinceBall.seconds(); }
    public boolean cantReach()            { return !feeding && waitTimer.seconds() > CANT_REACH_SECONDS; }

    /** One line for the driver screen. */
    public String status() {
        double vel = hw.shooterVelocity();
        if (feeding) return String.format("SHOOTING  %.0f / %d  (balls: %d)", vel, SHOT_VELOCITY, balls);
        if (cantReach()) return String.format("wheel CAN'T REACH %d (at %.0f) - D-pad down or new battery", SHOT_VELOCITY, vel);
        return String.format("waiting for wheel  %.0f / %d  (balls: %d)", vel, SHOT_VELOCITY, balls);
    }

    private void log(int target, double vel) {
        if (logTimer.milliseconds() < 50) return;
        logTimer.reset();
        RobotLog.ii("SHOT", "%.2f,%d,%.0f,%d,%.2f,%.2f", shotTimer.seconds(), target, vel,
                feeding ? 1 : 0, hw.batteryVoltage(), shooter.power());
    }
}
