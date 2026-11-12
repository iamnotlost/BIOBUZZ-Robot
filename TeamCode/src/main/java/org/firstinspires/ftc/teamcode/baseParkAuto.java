package org.firstinspires.ftc.teamcode;

// SAFE PARK auto (parkRed / parkBlue), for when the other alliance is strong and we just stay out of the way:
// start in our CORNER, drive straight off the wall, then sideways into our LOADING ZONE and stop there.
// LEAVE (3) + AUTO PARK (5) = 8 pts; two robots doing this = 16 pts = SWARM RP threshold.
//
// The park spot is taught with setupParkRed / setupParkBlue (INIT on the corner start, drive into the LOADING
// ZONE, press O). If it was never taught, the old numbers from settingsMenu are used instead
// ("Auto: leave distance", "Auto park: forward / sideways"; blue mirrors the sideways distance).
//
// Rules: G304 start touching the wall, not in the LOADING ZONE; LEAVE = not touching the wall and PARK = partly in
// the LOADING ZONE, both judged at the end of AUTO; G402 we stay on our own side, along our own wall.
public abstract class baseParkAuto extends baseAuto {
    public static double START_DELAY_S  = 0;
    public static double LEAVE_DISTANCE = 12;   // inches straight out from the wall (untaught version)
    public static double PARK_FORWARD   = 12;   // inches from start, + = away from the wall (untaught version)
    public static double PARK_SIDE      = 24;   // inches from start, + = robot's left (red, untaught version)
    public static double MIN_OFF_WALL   = 3;    // taught version: always get at least this far off the wall first
    public static double DRIVE_MAX_S    = 8;    // give up on a drive after this long (e.g. blocked)

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        setup();
        double[] park = autoSpots.park(alliance());
        boolean taught = autoSpots.isSet(park);

        while (opModeInInit()) {
            follower.update();
            telemetry.addData("Auto", "SAFE PARK " + alliance());
            telemetry.addData("Start", "CORNER start spot, touching the wall, not in the LOADING ZONE");
            telemetry.addData("Park at", taught ? autoSpots.text(park)
                    : "not taught - using settingsMenu numbers (run setupPark" + (alliance() == Alliance.RED ? "Red" : "Blue") + ")");
            double volts = hwManager.batteryVoltage();
            telemetry.addData("Battery", "%.1f V%s", volts, volts < 12.5 ? "  !! LOW - charge or swap" : "");
            if (!hwManager.missing.isEmpty()) telemetry.addData("!! Not found", hwManager.missingText());
            telemetry.addData("Odometry", "%s  (should stay 0, 0, 0 while still)", poseText(follower.getPose()));
            telemetry.update();
        }
        if (isStopRequested()) return;
        startTimer();

        waitSeconds(START_DELAY_S);
        if (taught) {
            // 1. straight off the wall (no scraping along it), 2. sideways into the LOADING ZONE.
            // "Off the wall" is the same way the taught park spot is: forwards if the robot starts facing the
            // field, BACKWARDS if it starts facing the wall (then the park spot's "ahead" is negative).
            double away = park[0] >= 0 ? Math.max(park[0], MIN_OFF_WALL) : Math.min(park[0], -MIN_OFF_WALL);
            step("1/2 straight off the wall");
            goTo("off the wall", new double[] {away, 0, 0}, DRIVE_MAX_S);
            step("2/2 into the LOADING ZONE");
            goTo("PARK spot", park, DRIVE_MAX_S);
        } else {
            step("1/2 straight off the wall (numbers)");
            goTo("off the wall", new double[] {LEAVE_DISTANCE, 0, 0}, DRIVE_MAX_S);
            step("2/2 into the LOADING ZONE (numbers)");
            goTo("PARK spot", new double[] {PARK_FORWARD, alliance().sign * PARK_SIDE, 0}, DRIVE_MAX_S);
        }
        finish();
    }
}
