package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Tells POLLEN (yellow), red NECTAR and blue NECTAR apart with a REV Color Sensor V3.
// Thresholds are first guesses - read the hue in hardwareTest with each ball and adjust.
public class elementSensor {
    public enum Element { NONE, POLLEN, RED_NECTAR, BLUE_NECTAR, UNKNOWN }

    public static float  GAIN             = 10;
    public static double PRESENT_CM       = 4.0;   // ball closer than this counts as present
    public static double PRESENT_BRIGHT   = 0.25;  // used when the sensor has no distance reading
    public static double YELLOW_HUE_MIN   = 40,  YELLOW_HUE_MAX = 90;
    public static double BLUE_HUE_MIN     = 190, BLUE_HUE_MAX   = 260;
    public static double RED_HUE_MAX      = 25,  RED_HUE_MIN_WRAP = 330;

    private final NormalizedColorSensor sensor;
    private final DistanceSensor distance;

    public elementSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.tryGet(NormalizedColorSensor.class, name);
        distance = sensor instanceof DistanceSensor ? (DistanceSensor) sensor : null;
        if (sensor != null) sensor.setGain(GAIN);
    }

    public boolean present() {
        return sensor != null;
    }

    public double hue() {
        if (sensor == null) return -1;
        NormalizedRGBA c = sensor.getNormalizedColors();
        double max = Math.max(c.red, Math.max(c.green, c.blue));
        double min = Math.min(c.red, Math.min(c.green, c.blue));
        double d = max - min;
        if (d <= 0) return 0;
        double h;
        if (max == c.red)        h = 60 * (((c.green - c.blue) / d) % 6);
        else if (max == c.green) h = 60 * ((c.blue - c.red) / d + 2);
        else                     h = 60 * ((c.red - c.green) / d + 4);
        return h < 0 ? h + 360 : h;
    }

    public double distanceCm() {
        return distance != null ? distance.getDistance(DistanceUnit.CM) : -1;
    }

    public boolean ballPresent() {
        if (sensor == null) return false;
        if (distance != null) return distanceCm() < PRESENT_CM;
        NormalizedRGBA c = sensor.getNormalizedColors();
        return Math.max(c.red, Math.max(c.green, c.blue)) > PRESENT_BRIGHT;
    }

    public Element read() {
        if (!ballPresent()) return Element.NONE;
        double h = hue();
        if (h >= YELLOW_HUE_MIN && h <= YELLOW_HUE_MAX) return Element.POLLEN;
        if (h >= BLUE_HUE_MIN && h <= BLUE_HUE_MAX)     return Element.BLUE_NECTAR;
        if (h <= RED_HUE_MAX || h >= RED_HUE_MIN_WRAP)  return Element.RED_NECTAR;
        return Element.UNKNOWN;
    }

    // G408: the other alliance's NECTAR must not stay in the robot
    public boolean isOpponentNectar(Alliance alliance) {
        Element e = read();
        return alliance == Alliance.RED ? e == Element.BLUE_NECTAR : e == Element.RED_NECTAR;
    }
}
