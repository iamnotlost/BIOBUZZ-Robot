package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Drive base only - works on any robot with the 4 drive motors, nothing else needed.
//   Left stick     move           Right stick X   turn
//   Right trigger  half speed     Left trigger    quarter speed
//   Square         field-centric / robot-centric
//   Options        reset heading (face the robot away from you first)
//   Hold D-pad     spin one wheel to check wiring: up=frontLeft, right=frontRight,
//                  down=backRight, left=backLeft. Each wheel should roll the robot forward.
@TeleOp(name="driveTest", group = "test")
public class driveTest extends OpMode {
    hwManager hwManager;
    drivetrain drivetrain;

    private static final double WHEEL_TEST_POWER = 0.3;

    private boolean fieldCentric = true;
    private boolean prevSquare, prevOptions;

    @Override
    public void init() {
        hwManager = new hwManager();
        hwManager.initHardware(hardwareMap);
        drivetrain = new drivetrain(hwManager);

        telemetry.addData("init", "complete");
        telemetry.addData("Missing", hwManager.missingText());
    }

    @Override
    public void loop() {
        hwManager.update();

        if (gamepad1.square && !prevSquare) fieldCentric = !fieldCentric;
        if (gamepad1.options && !prevOptions) drivetrain.resetYaw();
        prevSquare = gamepad1.square;
        prevOptions = gamepad1.options;

        String wheelTest = null;
        if (gamepad1.dpad_up)         wheelTest = spinOnly(hwManager.frontLeft, "frontLeft");
        else if (gamepad1.dpad_right) wheelTest = spinOnly(hwManager.frontRight, "frontRight");
        else if (gamepad1.dpad_down)  wheelTest = spinOnly(hwManager.backRight, "backRight");
        else if (gamepad1.dpad_left)  wheelTest = spinOnly(hwManager.backLeft, "backLeft");
        else {
            double x  = -gamepad1.left_stick_x;
            double y  =  gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            double speedMult = gamepad1.left_trigger > 0.05 ? 0.25 : gamepad1.right_trigger > 0.05 ? 0.5 : 1.0;

            if (fieldCentric) drivetrain.drive(x * speedMult, y * speedMult, rx * speedMult);
            else              drivetrain.driveRobotCentric(x * speedMult, y * speedMult, rx * speedMult);
        }

        telemetry.addData("Mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
        telemetry.addData("Wheel test", wheelTest != null ? wheelTest : "off");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(hwManager.getHeading()));
        if (hwManager.pinpoint != null) {
            telemetry.addData("Pinpoint X/Y (mm)", "%.0f / %.0f",
                    hwManager.pinpoint.getPosX(), hwManager.pinpoint.getPosY());
        }
        motorLine("frontLeft", hwManager.frontLeft);
        motorLine("frontRight", hwManager.frontRight);
        motorLine("backLeft", hwManager.backLeft);
        motorLine("backRight", hwManager.backRight);
        telemetry.addData("Missing", hwManager.missingText());
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
