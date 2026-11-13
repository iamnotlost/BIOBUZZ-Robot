package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.ElementSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * Every device on the robot, found by its name in the robot configuration ("MagisMain").
 * Every device is optional: anything missing stays null, the code using it does nothing, and its name is in
 * {@link #missing} (shown on the Driver Hub).
 *
 * Performance: call {@link #update} exactly once per loop. Each sensor is then read at most once per loop and
 * only if something needs it (motor speeds come from one bulk read per hub; the Pinpoint only when asked).
 */
public class RobotHardware {
    // Names in the robot configuration
    public static final String FRONT_LEFT = "frontLeft", FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backLeft", BACK_RIGHT = "backRight";
    public static final String INTAKE = "intake", FEEDER = "supply";
    public static final String SHOOTER = "shooter", SECOND_SHOOTER = "secondShooter";
    public static final String PINPOINT = "pinpoint", IMU_NAME = "imu", COLOR_SENSOR = "colorSensor";

    /** What the Pinpoint should read this loop. HEADING is a shorter, faster read than FULL. */
    public enum Odometry { NONE, HEADING, FULL }

    public final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public final DcMotorEx intake, feeder, shooter, secondShooter;
    public final GoBildaPinpointDriver pinpoint;
    public final IMU imu;
    public final ElementSensor elementSensor;
    public final List<String> missing = new ArrayList<>();

    private final HardwareMap hardwareMap;

    // Values read in update()
    private double shooterVelocity, intakeVelocity;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // Bulk reads: each hub sends all its motor readings in one message. AUTO re-reads when a value is
        // read a second time, and update() reads each value only once, so it is one message per hub per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeft  = driveMotor(FRONT_LEFT, DcMotor.Direction.FORWARD);
        frontRight = driveMotor(FRONT_RIGHT, DcMotor.Direction.REVERSE);
        backLeft   = driveMotor(BACK_LEFT, DcMotor.Direction.FORWARD);
        backRight  = driveMotor(BACK_RIGHT, DcMotor.Direction.REVERSE);

        intake = find(DcMotorEx.class, INTAKE);
        if (intake != null) {
            resetEncoder(intake, DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);   // FORWARD = pulls balls in
        }

        feeder = find(DcMotorEx.class, FEEDER);
        if (feeder != null) {
            resetEncoder(feeder, DcMotor.RunMode.RUN_USING_ENCODER);
            feeder.setDirection(DcMotorSimple.Direction.REVERSE);    // positive = balls go up
        }

        // Both shooter motors get the same plain power from the Shooter class; the shooter motor's encoder
        // measures the wheel speed
        shooter = find(DcMotorEx.class, SHOOTER);
        if (shooter != null) {
            resetEncoder(shooter, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        secondShooter = find(DcMotorEx.class, SECOND_SHOOTER);
        if (secondShooter != null) {
            secondShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            secondShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            secondShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        pinpoint = find(GoBildaPinpointDriver.class, PINPOINT);
        if (pinpoint != null) {
            // same as pedroPathing/Constants, so positions here match the autos
            pinpoint.setOffsets(-125.11, -51.03, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();
        }

        imu = find(IMU.class, IMU_NAME);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        }

        elementSensor = new ElementSensor(hardwareMap, COLOR_SENSOR);
        if (!elementSensor.present()) missing.add(COLOR_SENSOR);
    }

    /**
     * Starts a new loop: earlier readings are forgotten, the Pinpoint is read if asked. Call once at the start of
     * every loop. Motor speeds are then read the first time something asks for them, and only once per loop.
     */
    public void update(Odometry odometry) {
        shooterRead = false;
        intakeRead = false;
        if (odometry != Odometry.NONE) readPinpoint(odometry);
    }

    private boolean shooterRead, intakeRead;

    public double shooterVelocity() {
        if (!shooterRead) {
            shooterVelocity = shooter != null ? shooter.getVelocity() : 0;
            shooterRead = true;
        }
        return shooterVelocity;
    }

    public double intakeVelocity() {
        if (!intakeRead) {
            intakeVelocity = intake != null ? intake.getVelocity() : 0;
            intakeRead = true;
        }
        return intakeVelocity;
    }

    // ---------------- Pinpoint (heading) ----------------

    // The Pinpoint talks over an I2C cable. If that bus gets stuck (loose cable, motor noise) every read waits for
    // a timeout and the whole robot freezes. So after a failed read, or several slow ones in a row, stop talking
    // to it for the rest of the run and drive without it.
    private static final double PINPOINT_SLOW_MS = 40;
    private static final int PINPOINT_MAX_SLOW = 3;
    private boolean pinpointFailed = false;
    private int pinpointSlowReads = 0;

    private void readPinpoint(Odometry odometry) {
        if (pinpoint == null || pinpointFailed) return;
        long start = System.nanoTime();
        try {
            if (odometry == Odometry.HEADING) pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
            else pinpoint.update();
        } catch (RuntimeException e) {
            pinpointFailed = true;
            return;
        }
        double ms = (System.nanoTime() - start) / 1e6;
        pinpointSlowReads = ms > PINPOINT_SLOW_MS ? pinpointSlowReads + 1 : 0;
        if (pinpointSlowReads >= PINPOINT_MAX_SLOW) pinpointFailed = true;
    }

    public boolean pinpointFailed() {
        return pinpointFailed;
    }

    public boolean hasHeading() {
        return (pinpoint != null && !pinpointFailed) || imu != null;
    }

    /** Robot heading in radians: Pinpoint if working, otherwise the Control Hub IMU, otherwise 0. */
    public double heading() {
        if (pinpoint != null && !pinpointFailed) return pinpoint.getHeading(AngleUnit.RADIANS);
        if (imu != null) return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0;
    }

    public void resetHeading() {
        if (pinpoint != null && !pinpointFailed) pinpoint.resetPosAndIMU();
        if (imu != null) imu.resetYaw();
    }

    // ---------------- battery ----------------

    // Each voltage reading is a separate message to every hub, so read at most every 0.2 s
    private double voltage = 0;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    /** Lowest voltage any hub sees (the battery under load), or 0 if none report. */
    public double batteryVoltage() {
        if (voltage > 0 && voltageTimer.seconds() < 0.2) return voltage;
        voltageTimer.reset();
        double min = Double.MAX_VALUE;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        voltage = min == Double.MAX_VALUE ? 0 : min;
        return voltage;
    }

    public String missingText() {
        return missing.isEmpty() ? "none" : android.text.TextUtils.join(", ", missing);   // (String.join needs Android 8)
    }

    // ---------------- helpers ----------------

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

    private static void resetEncoder(DcMotorEx motor, DcMotor.RunMode mode) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(mode);
    }
}
