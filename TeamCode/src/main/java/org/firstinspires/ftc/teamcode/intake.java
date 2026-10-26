package org.firstinspires.ftc.teamcode;
public class intake {
    hwManager hwManager;

    public static int IN_VELOCITY  = 2000;
    public static int OUT_VELOCITY = -1000;

    public boolean running = false;
    boolean reverse = false;
    public intake(hwManager hwManager) {
        this.hwManager = hwManager;
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
