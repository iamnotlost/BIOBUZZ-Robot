package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Shoots balls one after another at baseTeleOp.SHOT_VELOCITY, the same way TeleOp triangle does:
// the first ball goes in only once the wheel is at speed and steady; after each ball the feeder pauses
// until the wheel is back at speed, so every ball flies the same. Used by the autos and setupAuto.
// Call start() once, then update() every loop, then stop().
public class burst {
    private final hwManager hwManager;
    private final shooter shooter;
    private final supply supply;
    private final intake intake;

    private boolean feeding, firstBall, wasInWindow, wasDip;
    private int balls;
    private int lastFeeder = 0;
    private boolean intakeOn = false;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime sinceBall = new ElapsedTime();

    public burst(hwManager hwManager, shooter shooter, supply supply, intake intake) {
        this.hwManager = hwManager;
        this.shooter = shooter;
        this.supply = supply;
        this.intake = intake;
    }

    public void start() {
        feeding = false;
        firstBall = true;
        wasInWindow = false;
        wasDip = true;
        balls = 0;
        sinceBall.reset();
    }

    public void update() {
        int target = baseTeleOp.SHOT_VELOCITY;
        shooter.spinTo(target);
        double vel = hwManager.shooterVelocity();

        boolean inWindow = Math.abs(vel - target) <= baseTeleOp.READY_TOLERANCE;
        if (inWindow && !wasInWindow) settleTimer.reset();
        wasInWindow = inWindow;
        boolean ready = inWindow && settleTimer.milliseconds() >= baseTeleOp.SETTLE_MS;

        // a ball hitting the wheel = a sudden drop below the shot speed
        boolean dip = vel < target - baseTeleOp.DROP_TOLERANCE;
        if (dip && !wasDip && !firstBall) {
            balls++;
            sinceBall.reset();
        }
        wasDip = dip;

        if (!feeding) {
            if (firstBall ? ready : inWindow) {
                if (firstBall) sinceBall.reset();
                feeding = true;
                firstBall = false;
            }
        } else if (baseTeleOp.WAIT_BETWEEN_BALLS && dip) {
            feeding = false;   // wait the moment it takes the wheel to get back to speed
        }

        shooter.setLearning(firstBall);
        shooter.update();
        setFeeder(feeding ? baseTeleOp.SHOT_FEED_VELOCITY : 0);
        setIntake(feeding);
    }

    public void stop() {
        feeding = false;
        shooter.turnOff();
        setFeeder(0);
        setIntake(false);
    }

    public int balls()                     { return balls; }
    public boolean started()               { return !firstBall; }   // the first ball has gone in
    public double secondsSinceLastBall()   { return sinceBall.seconds(); }
    public boolean feeding()               { return feeding; }

    private void setFeeder(int velocity) {
        if (velocity == lastFeeder) return;
        lastFeeder = velocity;
        supply.turnOff();
        if (velocity != 0) supply.turnOn(velocity);
    }

    private void setIntake(boolean on) {
        if (on == intakeOn) return;
        intakeOn = on;
        if (on) intake.turnOn();
        else    intake.stop();
    }
}
