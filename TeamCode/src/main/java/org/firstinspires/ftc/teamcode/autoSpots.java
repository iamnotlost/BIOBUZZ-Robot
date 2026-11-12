package org.firstinspires.ftc.teamcode;

// Where the shooting autos (shootRed / shootBlue) go. Saved by setupAutoRed / setupAutoBlue (drive the
// robot there and press a button), stored in the settings file, also editable in settingsMenu.
// Every spot is {x, y, heading} measured from where the robot STARTS:
//   x = inches straight ahead, y = inches to the robot's left, heading = degrees turned left (+) / right (-).
// Red and blue each have their own spots, so nothing assumes the field is mirrored.
public class autoSpots {
    public static final double[] RED_SHOOT  = new double[3];
    public static final double[] RED_END    = new double[3];
    public static final double[] BLUE_SHOOT = new double[3];
    public static final double[] BLUE_END   = new double[3];

    public static double[] shoot(Alliance a) { return a == Alliance.RED ? RED_SHOOT : BLUE_SHOOT; }
    public static double[] end(Alliance a)   { return a == Alliance.RED ? RED_END : BLUE_END; }

    public static boolean isSet(double[] spot) {
        return spot[0] != 0 || spot[1] != 0 || spot[2] != 0;
    }

    public static String text(double[] spot) {
        if (!isSet(spot)) return "NOT SET";
        return String.format("%.0f in ahead, %.0f in left, turned %.0f deg", spot[0], spot[1], spot[2]);
    }
}
