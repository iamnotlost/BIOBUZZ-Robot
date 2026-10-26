package org.firstinspires.ftc.teamcode;
public class intake {
    hwManager hwManager;

    public boolean running = false;
    boolean reverse = false;
    public intake(hwManager hwManager) {
        this.hwManager = hwManager;
    }
    public void turnOn() {
        if (running) return;
        hwManager.intake.setVelocity(2000);
        running = true;
    }
    public void turnOff() {
        if (!running) return;
        hwManager.intake.setVelocity(0);
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
        hwManager.intake.setVelocity(0);
        running = false;
        reverse = false;
    }
    public void toggleReverse() {
        if (reverse) {
            hwManager.intake.setVelocity(0);
            reverse = false;
        } else {
            hwManager.intake.setVelocity(-1000);
            reverse = true;
        }
        running = false;
    }
}
