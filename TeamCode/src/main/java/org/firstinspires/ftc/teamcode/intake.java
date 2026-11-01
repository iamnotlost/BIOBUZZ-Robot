package org.firstinspires.ftc.teamcode;
public class intake {
    hwManager hwManager;

    // Ticks/s. Adjustable while driving (gamepad 2 D-pad up/down); the value shows on telemetry
    public static int IN_VELOCITY  = 300;
    public static int OUT_VELOCITY = -300;
    public static final int SPEED_STEP = 100;
    public static final int MIN_SPEED  = 100;
    public static final int MAX_SPEED  = 2500;

    public boolean running = false;
    boolean reverse = false;
    public intake(hwManager hwManager) {
        this.hwManager = hwManager;
    }
    // Changes both directions by the same step and applies it right away if the intake is running
    public void adjustSpeed(int steps) {
        IN_VELOCITY = Math.max(MIN_SPEED, Math.min(MAX_SPEED, IN_VELOCITY + steps * SPEED_STEP));
        OUT_VELOCITY = -Math.max(MIN_SPEED, Math.min(MAX_SPEED, -OUT_VELOCITY + steps * SPEED_STEP));
        if (running) setVelocity(IN_VELOCITY);
        if (reverse) setVelocity(OUT_VELOCITY);
    }
    private void setVelocity(int velocity) {
        if (hwManager.intake != null) hwManager.intake.setVelocity(velocity);
    }
    public void turnOn() {
        if (running) return;
        setVelocity(IN_VELOCITY);
        running = true;
        reverse = false;
    }
    public void turnOff() {
        if (!running) return;
        setVelocity(0);
        running = false;
    }
    public void toggle() {
        if (running) {
            turnOff();
        } else {
            turnOn();
        }
    }
    public void stop() {
        setVelocity(0);
        running = false;
        reverse = false;
    }
    public void reverseOn() {
        setVelocity(OUT_VELOCITY);
        reverse = true;
        running = false;
    }
    public boolean isReversing() {
        return reverse;
    }
    public void toggleReverse() {
        if (reverse) {
            stop();
        } else {
            reverseOn();
        }
    }
}
