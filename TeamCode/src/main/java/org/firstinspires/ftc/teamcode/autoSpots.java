package org.firstinspires.ftc.teamcode;

// Where the shooting autos (shootRed / shootBlue) go. Saved by setupAutoRed / setupAutoBlue (drive the
// robot there and press a button), stored in the settings file, also editable in settingsMenu.
// Every spot is {x, y, heading} measured from where the robot STARTS:
//   x = inches straight ahead, y = inches to the robot's left, heading = degrees turned left (+) / right (-).
// Red and blue each have their own spots, so nothing assumes the field is mirrored.
//
// Two PLANS per alliance (A and B), each with its own start spot, so if our alliance partner needs our usual
// start spot we pick the other plan at INIT (D-pad left/right). The other alliance can never take our spot:
// G304 makes every robot start on its own side.
// A plan is taught once its END spot is saved. A SHOOT spot of 0,0,0 means "shoot from the start spot".
public class autoSpots {
    public static final double[] RED_SHOOT    = new double[3];   // plan A
    public static final double[] RED_END      = new double[3];
    public static final double[] BLUE_SHOOT   = new double[3];
    public static final double[] BLUE_END     = new double[3];
    public static final double[] RED_SHOOT_B  = new double[3];   // plan B
    public static final double[] RED_END_B    = new double[3];
    public static final double[] BLUE_SHOOT_B = new double[3];
    public static final double[] BLUE_END_B   = new double[3];
    // Safe park auto (parkRed / parkBlue), taught from the CORNER start with setupParkRed / setupParkBlue
    public static final double[] RED_PARK     = new double[3];
    public static final double[] BLUE_PARK    = new double[3];

    // The plan chosen at INIT (0 = A, 1 = B); kept until the robot app restarts
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

    public static double[] park(Alliance a) { return a == Alliance.RED ? RED_PARK : BLUE_PARK; }

    public static boolean isSet(double[] spot) {
        return spot[0] != 0 || spot[1] != 0 || spot[2] != 0;
    }

    // a plan is ready once its END spot was saved (the END spot can never be the start spot)
    public static boolean taught(Alliance a, int p) {
        return isSet(end(a, p));
    }

    public static String text(double[] spot) {
        if (!isSet(spot)) return "NOT SET";
        return String.format("%.0f in ahead, %.0f in left, turned %.0f deg", spot[0], spot[1], spot[2]);
    }

    // ---- Path points: places the robot drives THROUGH on the way to a spot, so it doesn't go in a straight line
    // (around the HIVE, the partner robot, ...). Taught with L1 in setupAuto / setupPark, in order, measured from
    // START like the spots. Routes: "<red|blue><A|B>Shoot" (start -> shoot), "<red|blue><A|B>End" (shoot -> end),
    // "<red|blue>Park" (start -> safe park spot). Saved in the settings file as "path.<route>" = "x,y,h;x,y,h".
    public static final int MAX_POINTS = 5;
    private static final java.util.Map<String, java.util.List<double[]>> POINTS = new java.util.HashMap<>();

    public static String route(Alliance a, int p, boolean toShoot) {
        return (a == Alliance.RED ? "red" : "blue") + planName(p) + (toShoot ? "Shoot" : "End");
    }

    public static String parkRoute(Alliance a) {
        return (a == Alliance.RED ? "red" : "blue") + "Park";
    }

    public static java.util.List<double[]> points(String route) {
        java.util.List<double[]> list = POINTS.get(route);
        return list != null ? list : new java.util.ArrayList<double[]>();
    }

    public static void setPoints(String route, java.util.List<double[]> points) {
        java.util.List<double[]> copy = new java.util.ArrayList<>();
        for (double[] pt : points) copy.add(pt.clone());
        POINTS.put(route, copy);
    }

    public static java.util.Set<String> routes() {
        return POINTS.keySet();
    }

    public static String encodePoints(String route) {
        StringBuilder s = new StringBuilder();
        for (double[] pt : points(route)) {
            if (s.length() > 0) s.append(';');
            s.append(Math.round(pt[0])).append(',').append(Math.round(pt[1])).append(',').append(Math.round(pt[2]));
        }
        return s.toString();
    }

    public static void decodePoints(String route, String text) {
        java.util.List<double[]> list = new java.util.ArrayList<>();
        if (text != null && !text.trim().isEmpty()) {
            for (String part : text.split(";")) {
                String[] v = part.split(",");
                if (v.length != 3) continue;
                try {
                    list.add(new double[] {Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2])});
                } catch (NumberFormatException ignored) { }
            }
        }
        POINTS.put(route, list);
    }

    public static String shootText(Alliance a, int p) {
        double[] s = shoot(a, p);
        if (!isSet(s)) return taught(a, p) ? "the START spot (shoots without driving)" : "NOT SET";
        return text(s);
    }
}
