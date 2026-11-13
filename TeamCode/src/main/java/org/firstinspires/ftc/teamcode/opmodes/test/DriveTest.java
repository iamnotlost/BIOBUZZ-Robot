package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Drive base only - works with just the 4 drive motors.
 * <pre>
 *   left stick    move              right stick X  turn
 *   right trigger half speed        left trigger   quarter speed
 *   square        field / robot-centric
 *   options       reset heading (face the robot away from you first)
 *   hold D-pad    spin one wheel: up = frontLeft, right = frontRight, down = backRight, left = backLeft.
 *                 Each wheel should roll the robot forward.
 * </pre>
 */
@TeleOp(name = "driveTest", group = "test")
public class DriveTest extends OpMode {
    private static final double WHEEL_TEST_POWER = 0.3;

    private RobotHardware hw;
    private Drivetrain drivetrain;
    private boolean fieldCentric = true;

    @Override
    public void init() {
        hw = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hw);
        telemetry.addData("Init", "complete");
        telemetry.addData("Missing", hw.missingText());
    }

    @Override
    public void loop() {
        hw.update(RobotHardware.Odometry.FULL);
        if (gamepad1.squareWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.optionsWasPressed()) hw.resetHeading();

        String wheelTest = null;
        if (gamepad1.dpad_up)         wheelTest = spinOnly(hw.frontLeft, "frontLeft");
        else if (gamepad1.dpad_right) wheelTest = spinOnly(hw.frontRight, "frontRight");
        else if (gamepad1.dpad_down)  wheelTest = spinOnly(hw.backRight, "backRight");
        else if (gamepad1.dpad_left)  wheelTest = spinOnly(hw.backLeft, "backLeft");
        else {
            double speed = gamepad1.left_trigger > 0.05 ? 0.25 : gamepad1.right_trigger > 0.05 ? 0.5 : 1.0;
            double top = Drivetrain.MAX_SPEED * speed, turnTop = Drivetrain.TURN_SPEED * speed;
            double x = -Drivetrain.shape(gamepad1.left_stick_x, top, Drivetrain.MIN_DRIVE_POWER);
            double y = Drivetrain.shape(gamepad1.left_stick_y, top, Drivetrain.MIN_DRIVE_POWER);
            double rx = -Drivetrain.shape(gamepad1.right_stick_x, turnTop, Drivetrain.MIN_TURN_POWER);
            drivetrain.drive(x, y, rx, fieldCentric ? hw.heading() : 0);
        }

        telemetry.addData("Mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
        telemetry.addData("Wheel test", wheelTest != null ? wheelTest : "off");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(hw.heading()));
        if (hw.pinpoint != null) {
            telemetry.addData("Pinpoint X/Y (mm)", "%.0f / %.0f",
                    hw.pinpoint.getPosX(DistanceUnit.MM), hw.pinpoint.getPosY(DistanceUnit.MM));
        }
        motorLine("frontLeft", hw.frontLeft);
        motorLine("frontRight", hw.frontRight);
        motorLine("backLeft", hw.backLeft);
        motorLine("backRight", hw.backRight);
        telemetry.addData("Missing", hw.missingText());
        telemetry.update();
    }

    private String spinOnly(DcMotorEx motor, String name) {
        drivetrain.stop();
        if (motor == null) return name + " NOT FOUND";
        motor.setPower(WHEEL_TEST_POWER);
        return name;
    }

    private void motorLine(String name, DcMotorEx motor) {
        if (motor == null) telemetry.addData(name, "MISSING");
        else telemetry.addData(name, "power %.2f  encoder %d", motor.getPower(), motor.getCurrentPosition());
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
