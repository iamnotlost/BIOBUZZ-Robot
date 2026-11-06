package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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

        // Bulk reads: each hub sends all its motor/sensor readings in one message per loop instead of
        // one message per reading, so the loop runs faster and steadier
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeft = driveMotor(FRONT_LEFT, DcMotor.Direction.FORWARD);
        frontRight = driveMotor(FRONT_RIGHT, DcMotor.Direction.REVERSE);
        backLeft = driveMotor(BACK_LEFT, DcMotor.Direction.FORWARD);
        backRight = driveMotor(BACK_RIGHT, DcMotor.Direction.REVERSE);

        intake = find(DcMotorEx.class, INTAKE);
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotor.Direction.FORWARD); // FORWARD = pulls balls in on this robot
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

    // The Pinpoint talks over an I2C cable. If that bus gets stuck (loose or damaged cable, motor noise),
    // every read waits for a timeout and the whole robot freezes. So after a failed read, or several slow
    // ones in a row, we stop talking to it for the rest of the run and drive without it.
    public boolean pinpointFailed = false;
    private static final double PINPOINT_SLOW_MS = 40;   // a normal read takes a few ms
    private static final int    PINPOINT_MAX_SLOW = 3;
    private int pinpointSlowReads = 0;

    // Call once per loop
    public void update() {
        if (pinpoint == null || pinpointFailed) return;
        long start = System.nanoTime();
        try {
            pinpoint.update();
        } catch (RuntimeException e) {
            pinpointFailed = true;
            return;
        }
        double ms = (System.nanoTime() - start) / 1e6;
        pinpointSlowReads = ms > PINPOINT_SLOW_MS ? pinpointSlowReads + 1 : 0;
        if (pinpointSlowReads >= PINPOINT_MAX_SLOW) pinpointFailed = true;
    }

    public boolean hasHeading() {
        return (pinpoint != null && !pinpointFailed) || imu != null;
    }

    // Robot heading in radians: Pinpoint if working, otherwise the Control Hub IMU, otherwise 0
    public double getHeading() {
        if (pinpoint != null && !pinpointFailed) return pinpoint.getHeading();
        if (imu != null) return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0;
    }

    public void resetHeading() {
        if (pinpoint != null && !pinpointFailed) pinpoint.resetPosAndIMU();
        if (imu != null) imu.resetYaw();
    }

    public boolean buttonPressed() {
        return button != null && !button.getState();
    }

    // Lowest voltage any hub sees (the battery under load), or 0 if none report
    public double batteryVoltage() {
        double min = Double.MAX_VALUE;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return min == Double.MAX_VALUE ? 0 : min;
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
