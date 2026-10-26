package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class drivetrain {
    hwManager hwManager;
    private static final double STRAFE_CORRECTION = 1.1;

    public drivetrain(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    public void resetYaw() {
        hwManager.resetHeading();
    }

    // Field-centric: x/y are relative to the field, rx turns the robot
    public void drive(double x, double y, double rx) {
        double botHeading = hwManager.getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        driveRobotCentric(rotX, rotY, rx);
    }

    // Robot-centric: forward is wherever the robot faces
    public void driveRobotCentric(double x, double y, double rx) {
        x = x * STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setPower(hwManager.frontLeft,  (y + x + rx) / denominator);
        setPower(hwManager.backLeft,   (y - x + rx) / denominator);
        setPower(hwManager.frontRight, (y - x - rx) / denominator);
        setPower(hwManager.backRight,  (y + x - rx) / denominator);
    }

    public void stop() {
        driveRobotCentric(0, 0, 0);
    }

    private void setPower(DcMotorEx motor, double power) {
        if (motor != null) motor.setPower(power);
    }
}
