package org.firstinspires.ftc.teamcode;

public class shooter {
    hwManager hwManager;

    // Ticks/s - tune once the launcher exists
    public static int FAR_VELOCITY    = 1850;
    public static int CLOSE_VELOCITY  = 1205;
    public static int MIDDLE_VELOCITY = 1150;

    public shooter(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    public int currentMode = 0;
    public void update() {
        if (hwManager.shooter == null || hwManager.secondShooter == null) return;
        hwManager.secondShooter.setPower(hwManager.shooter.getPower());
    }

    private void setVelocity(int velocity) {
        if (hwManager.shooter != null) hwManager.shooter.setVelocity(velocity);
    }

    public void turnOff() {
        setVelocity(0);
        if (hwManager.secondShooter != null) hwManager.secondShooter.setPower(0);
        currentMode = 0;
    }

    public void shootFar() {
        if (currentMode == 1) { turnOff(); return; }
        setVelocity(FAR_VELOCITY);
        currentMode = 1;
    }

    public void shootClose() {
        if (currentMode == 2) { turnOff(); return; }
        setVelocity(CLOSE_VELOCITY);
        currentMode = 2;
    }

    public void shootMiddle() {
        if (currentMode == 3) { turnOff(); return; }
        setVelocity(MIDDLE_VELOCITY);
        currentMode = 3;
    }

    public void shootCustom(int speed) {
        if (currentMode == 4) {turnOff(); return;}
        setVelocity(speed);
        currentMode = 4;
    }
}
