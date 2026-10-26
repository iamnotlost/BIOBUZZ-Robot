package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class supply {
    private final hwManager hwManager;
    public boolean running = false;
    public boolean isOn = false;

    private static final long STOP_DELAY_MS = 1;
    private final ElapsedTime stopTimer = new ElapsedTime();
    private boolean stopArmed = false;

    public supply(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    public void turnOn() {
        if (running) return;
        hwManager.supply.setVelocity(1400);
        running = true;
        stopArmed = false;
    }
    public void turnOn(int velocity) {
        if (running) return;
        hwManager.supply.setVelocity(velocity);
        running = true;
        stopArmed = false;
    }
    public void turnOnSlow() {
        if (running) return;
        hwManager.supply.setVelocity(1400);
        running = true;
        stopArmed = false;
    }
    public void turnOff() {
        if (!running) return;
        hwManager.supply.setVelocity(0);
        running = false;
    }
    public void toggle() {
        if (running) {
            turnOff();
        } else {
            turnOn(1400);
        }
    }

    public void stopOnButton(boolean triggered) {
        if (triggered && !stopArmed) {
            stopArmed = true;
            stopTimer.reset();
        }
        if (stopArmed && stopTimer.milliseconds() >= STOP_DELAY_MS) {
            turnOff();
            stopArmed = false;
        }
    }
}
