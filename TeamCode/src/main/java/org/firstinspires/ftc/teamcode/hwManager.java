package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

// Every device is optional: anything not in the robot configuration stays null and the code
// that uses it does nothing. Check the "Missing" telemetry line to see what wasn't found.
public class hwManager {
    // Names to use in the Driver Station robot configuration
    public static final String FRONT_LEFT     = "frontLeft";
    public static final String FRONT_RIGHT    = "frontRight";
    public static final String BACK_LEFT      = "backLeft";
    public static final String BACK_RIGHT     = "backRight";
    public static final String INTAKE         = "intake";
    public static final String SHOOTER        = "shooter";
    public static final String SECOND_SHOOTER = "secondShooter";
    public static final String SUPPLY         = "supply";
    public static final String BUTTON         = "button";
    public static final String COLOR_SENSOR   = "colorSensor";
    public static final String PINPOINT       = "pinpoint";
    public static final String IMU_NAME       = "imu";

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx intake, shooter, secondShooter, supply;
    GoBildaPinpointDriver pinpoint;
    IMU imu;
    DigitalChannel button;
    elementSensor elementSensor;

    public final List<String> missing = new ArrayList<>();
    private HardwareMap hardwareMap;

    public void initHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        missing.clear();

        frontLeft = driveMotor(FRONT_LEFT, DcMotor.Direction.FORWARD);
        frontRight = driveMotor(FRONT_RIGHT, DcMotor.Direction.REVERSE);
        backLeft = driveMotor(BACK_LEFT, DcMotor.Direction.FORWARD);
        backRight = driveMotor(BACK_RIGHT, DcMotor.Direction.REVERSE);

        intake = find(DcMotorEx.class, INTAKE);
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotor.Direction.REVERSE);
        }

        shooter = find(DcMotorEx.class, SHOOTER);
        if (shooter != null) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            shooter.setVelocityPIDFCoefficients(25, 0, 0, 12.8);
        }

        secondShooter = find(DcMotorEx.class, SECOND_SHOOTER);
        if (secondShooter != null) {
            secondShooter.setDirection(DcMotorSimple.Direction.FORWARD);
            secondShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        supply = find(DcMotorEx.class, SUPPLY);
        if (supply != null) {
            supply.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            supply.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            supply.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        button = find(DigitalChannel.class, BUTTON);
        if (button != null) button.setMode(DigitalChannel.Mode.INPUT);

        elementSensor = new elementSensor(hardwareMap, COLOR_SENSOR);
        if (!elementSensor.present()) missing.add(COLOR_SENSOR);

        imu = find(IMU.class, IMU_NAME);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        }

        pinpoint = find(GoBildaPinpointDriver.class, PINPOINT);
        if (pinpoint != null) {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setOffsets(-125.11, -51.03);
            pinpoint.resetPosAndIMU();
        }
    }

    private <T> T find(Class<T> type, String name) {
        T device = hardwareMap.tryGet(type, name);
        if (device == null) missing.add(name);
        return device;
    }

    private DcMotorEx driveMotor(String name, DcMotor.Direction direction) {
        DcMotorEx motor = find(DcMotorEx.class, name);
        if (motor != null) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(direction);
        }
        return motor;
    }

    // Call once per loop
    public void update() {
        if (pinpoint != null) pinpoint.update();
    }

    // Robot heading in radians: Pinpoint if present, otherwise the Control Hub IMU
    public double getHeading() {
        if (pinpoint != null) return pinpoint.getHeading();
        if (imu != null) return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0;
    }

    public void resetHeading() {
        if (pinpoint != null) pinpoint.resetPosAndIMU();
        if (imu != null) imu.resetYaw();
    }

    public boolean buttonPressed() {
        return button != null && !button.getState();
    }

    public double shooterVelocity() {
        return shooter != null ? shooter.getVelocity() : 0;
    }

    public String missingText() {
        if (missing.isEmpty()) return "none";
        StringBuilder text = new StringBuilder();
        for (String name : missing) {
            if (text.length() > 0) text.append(", ");
            text.append(name);
        }
        return text.toString();
    }
}
