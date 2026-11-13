package org.firstinspires.ftc.teamcode.util;

/** Small angle helpers. */
public final class Angles {
    private Angles() { }

    /** Wraps an angle in radians to (-pi, pi], i.e. the short way round. */
    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /** Wraps an angle in degrees to (-180, 180]. */
    public static double wrapDegrees(double degrees) {
        return Math.toDegrees(wrap(Math.toRadians(degrees)));
    }

    public static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
