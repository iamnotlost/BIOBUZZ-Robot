package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Where the autos go, taught with setupAuto / setupPark (drive the robot there and press a button) and saved in
 * the settings file. Every spot is {x, y, heading} measured from where the robot STARTS:
 * x = inches straight ahead, y = inches to the robot's left, heading = degrees turned left (+) / right (-).
 * Red and blue have their own spots (nothing assumes the field is mirrored).
 *
 * Two PLANS per alliance (A, B) with different start spots, picked with the D-pad at INIT, for when the alliance
 * partner needs our usual spot (the other alliance can't take it: G304, everyone starts on their own side).
 * A plan is taught once its END spot is saved. A SHOOT spot of 0,0,0 = shoot from the start spot.
 *
 * Path points: places the robot drives THROUGH on the way (around the HIVE, the partner...), taught with L1.
 * Routes: "redAShoot" (start -> shoot), "redAEnd" (shoot -> end), "redPark" (start -> safe park), ...
 */
public final class AutoSpots {
    private AutoSpots() { }

    public static final double[] RED_SHOOT    = new double[3];   // plan A
    public static final double[] RED_END      = new double[3];
    public static final double[] BLUE_SHOOT   = new double[3];
    public static final double[] BLUE_END     = new double[3];
    public static final double[] RED_SHOOT_B  = new double[3];   // plan B
    public static final double[] RED_END_B    = new double[3];
    public static final double[] BLUE_SHOOT_B = new double[3];
    public static final double[] BLUE_END_B   = new double[3];
    public static final double[] RED_PARK     = new double[3];   // safe park, from the CORNER start
    public static final double[] BLUE_PARK    = new double[3];

    public static final int MAX_POINTS = 5;
    private static final Map<String, List<double[]>> POINTS = new HashMap<>();

    /** The plan chosen at INIT (0 = A, 1 = B); kept until the robot app restarts. */
    public static int plan = 0;

    public static String planName(int p) { return p == 0 ? "A" : "B"; }

    public static double[] shoot(Alliance a, int p) {
        if (a == Alliance.RED) return p == 0 ? RED_SHOOT : RED_SHOOT_B;
        return p == 0 ? BLUE_SHOOT : BLUE_SHOOT_B;
    }

    public static double[] end(Alliance a, int p) {
        if (a == Alliance.RED) return p == 0 ? RED_END : RED_END_B;
        return p == 0 ? BLUE_END : BLUE_END_B;
    }

    public static double[] park(Alliance a) {
        return a == Alliance.RED ? RED_PARK : BLUE_PARK;
    }

    public static boolean isSet(double[] spot) {
        return spot[0] != 0 || spot[1] != 0 || spot[2] != 0;
    }

    /** A plan is ready once its END spot was saved (the END spot can never be the start spot). */
    public static boolean taught(Alliance a, int p) {
        return isSet(end(a, p));
    }

    public static String text(double[] spot) {
        if (!isSet(spot)) return "NOT SET";
        return String.format("%.0f in ahead, %.0f in left, turned %.0f deg", spot[0], spot[1], spot[2]);
    }

    public static String shootText(Alliance a, int p) {
        double[] s = shoot(a, p);
        if (!isSet(s)) return taught(a, p) ? "the START spot (shoots without driving)" : "NOT SET";
        return text(s);
    }

    // ---------------- path points ----------------

    public static String route(Alliance a, int p, boolean toShoot) {
        return a.name().toLowerCase() + planName(p) + (toShoot ? "Shoot" : "End");
    }

    public static String parkRoute(Alliance a) {
        return a.name().toLowerCase() + "Park";
    }

    public static List<double[]> points(String route) {
        List<double[]> list = POINTS.get(route);
        return list != null ? list : new ArrayList<double[]>();
    }

    public static void setPoints(String route, List<double[]> points) {
        List<double[]> copy = new ArrayList<>();
        for (double[] pt : points) copy.add(pt.clone());
        POINTS.put(route, copy);
    }

    public static Set<String> routes() {
        return POINTS.keySet();
    }

    /** "x,y,h;x,y,h" for the settings file. */
    public static String encodePoints(String route) {
        StringBuilder s = new StringBuilder();
        for (double[] pt : points(route)) {
            if (s.length() > 0) s.append(';');
            s.append(Math.round(pt[0])).append(',').append(Math.round(pt[1])).append(',').append(Math.round(pt[2]));
        }
        return s.toString();
    }

    public static void decodePoints(String route, String text) {
        List<double[]> list = new ArrayList<>();
        if (text != null) {
            for (String part : text.split(";")) {
                String[] v = part.trim().split(",");
                if (v.length != 3) continue;
                try {
                    list.add(new double[] {Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2])});
                } catch (NumberFormatException ignored) {
                    // a broken point is skipped
                }
            }
        }
        POINTS.put(route, list);
    }
}
