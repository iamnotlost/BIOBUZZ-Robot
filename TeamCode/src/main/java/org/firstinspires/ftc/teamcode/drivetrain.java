package org.firstinspires.ftc.teamcode;

public class drivetrain {
    hwManager hwManager;
    private static final double STRAFE_CORRECTION = 1.1;

    public drivetrain(hwManager hwManager) {
        this.hwManager = hwManager;
    }

    public void resetYaw() {
        hwManager.pinpoint.resetPosAndIMU();
    }

    public void drive(double x, double y, double rx) {
        x = x * STRAFE_CORRECTION;

        double botHeading = hwManager.pinpoint.getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        hwManager.frontLeft.setPower(frontLeftPower);
        hwManager.frontRight.setPower(frontRightPower);
        hwManager.backLeft.setPower(backLeftPower);
        hwManager.backRight.setPower(backRightPower);
    }
}
