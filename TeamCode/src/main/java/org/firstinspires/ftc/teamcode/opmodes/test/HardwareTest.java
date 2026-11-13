package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs any motor or servo in the robot configuration, one at a time, whatever it is named - to check ports,
 * directions and encoders. D-pad up/down picks a device, left stick Y runs it, triangle = servo to 0.5.
 * A servo only moves once you touch the stick or triangle, so selecting one never moves it.
 */
@TeleOp(name = "hardwareTest", group = "test")
public class HardwareTest extends OpMode {
    private RobotHardware hw;
    private final List<HardwareDevice> devices = new ArrayList<>();
    private final List<String> names = new ArrayList<>();
    private int selected = 0;
    private double servoPosition = 0.5;
    private boolean servoArmed = false;

    @Override
    public void init() {
        hw = new RobotHardware(hardwareMap);
        add(hardwareMap.getAll(DcMotorEx.class));
        add(hardwareMap.getAll(CRServo.class));
        add(hardwareMap.getAll(Servo.class));
        telemetry.addData("Devices found", devices.size());
        telemetry.addData("Missing (by name)", hw.missingText());
    }

    private void add(List<? extends HardwareDevice> list) {
        for (HardwareDevice d : list) {
            devices.add(d);
            names.add(hardwareMap.getNamesOf(d).iterator().next());
        }
    }

    @Override
    public void loop() {
        hw.update(RobotHardware.Odometry.FULL);
        if (devices.isEmpty()) {
            telemetry.addData("No motors or servos", "check the robot configuration");
            telemetry.update();
            return;
        }
        if (gamepad1.dpadUpWasPressed()) select(selected - 1);
        if (gamepad1.dpadDownWasPressed()) select(selected + 1);

        HardwareDevice device = devices.get(selected);
        double stick = -gamepad1.left_stick_y;
        if (device instanceof DcMotorEx) {
            DcMotorEx motor = (DcMotorEx) device;
            motor.setPower(stick);
            telemetry.addData("Type", "motor (port " + motor.getPortNumber() + ")");
            telemetry.addData("Power", "%.2f", stick);
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.addData("Velocity (ticks/s)", "%.0f", motor.getVelocity());
        } else if (device instanceof CRServo) {
            CRServo servo = (CRServo) device;
            servo.setPower(stick);
            telemetry.addData("Type", "CR servo (port " + servo.getPortNumber() + ")");
            telemetry.addData("Power", "%.2f", stick);
        } else if (device instanceof Servo) {
            Servo servo = (Servo) device;
            if (gamepad1.triangle || Math.abs(stick) > 0.1) servoArmed = true;
            if (servoArmed) {
                if (gamepad1.triangle) servoPosition = 0.5;
                servoPosition = Math.max(0, Math.min(1, servoPosition + stick * 0.01));
                servo.setPosition(servoPosition);
            }
            telemetry.addData("Type", "servo (port " + servo.getPortNumber() + ")");
            telemetry.addData("Position", servoArmed ? String.format("%.3f", servoPosition) : "not moved yet");
        }

        telemetry.addData("Selected", "%s  (%d of %d)", names.get(selected), selected + 1, devices.size());
        telemetry.addLine();
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(hw.heading()));
        if (hw.pinpoint != null) {
            telemetry.addData("Pinpoint X/Y (mm)", "%.0f / %.0f",
                    hw.pinpoint.getPosX(DistanceUnit.MM), hw.pinpoint.getPosY(DistanceUnit.MM));
        }
        if (hw.elementSensor.present()) {
            telemetry.addData("Color hue", "%.0f", hw.elementSensor.hue());
            telemetry.addData("Color distance (cm)", "%.1f", hw.elementSensor.distanceCm());
            telemetry.addData("Ball", hw.elementSensor.read());
        }
        telemetry.addData("Missing (by name)", hw.missingText());
        telemetry.update();
    }

    // Stops the old device before switching so nothing keeps running unattended
    private void select(int index) {
        HardwareDevice old = devices.get(selected);
        if (old instanceof DcMotorEx) ((DcMotorEx) old).setPower(0);
        if (old instanceof CRServo) ((CRServo) old).setPower(0);
        selected = (index + devices.size()) % devices.size();
        servoArmed = false;
        if (devices.get(selected) instanceof Servo) {
            servoPosition = ((Servo) devices.get(selected)).getPosition();
            if (Double.isNaN(servoPosition)) servoPosition = 0.5;
        }
    }

    @Override
    public void stop() {
        for (HardwareDevice d : devices) {
            if (d instanceof DcMotorEx) ((DcMotorEx) d).setPower(0);
            if (d instanceof CRServo) ((CRServo) d).setPower(0);
        }
    }
}
