package org.firstinspires.ftc.teamcode;

public class shooter {
    hwManager hwManager;

    public shooter(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    public int currentMode = 0;
    public void update() {
        hwManager.secondShooter.setPower(hwManager.shooter.getPower());
    }

    public void turnOff() {
        hwManager.shooter.setVelocity(0);
        currentMode = 0;
    }

    public void shootFar() {
        if (currentMode == 1) { turnOff(); return; }
        hwManager.shooter.setVelocity(1850);
        currentMode = 1;
    }

    public void shootClose() {
        if (currentMode == 2) { turnOff(); return; }
        hwManager.shooter.setVelocity(1205);
        currentMode = 2;
    }

    public void shootMiddle() {
        if (currentMode == 3) { turnOff(); return; }
        hwManager.shooter.setVelocity(1150);
        currentMode = 3;
    }

    public void shootCustom(int speed) {
        if (currentMode == 4) {turnOff(); return;}
        hwManager.shooter.setVelocity(speed);
        currentMode = 4;
    }
}