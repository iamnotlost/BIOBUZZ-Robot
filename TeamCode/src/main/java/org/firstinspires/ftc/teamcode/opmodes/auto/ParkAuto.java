package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSpots;

import java.util.List;

/**
 * SAFE PARK auto, for when the other alliance is strong and we stay out of the way: start in our CORNER, then
 * into our LOADING ZONE (LEAVE 3 + PARK 5; two robots doing this = 16 = SWARM RP threshold).
 * The park spot (and path points) are taught with setupParkRed / setupParkBlue. Without path points it first
 * drives straight off the wall (the same way as the park spot: the robot may start facing the wall), then to the
 * spot. Never taught: the settingsMenu numbers are used (blue mirrors the sideways distance).
 */
public abstract class ParkAuto extends BaseAuto {
    public static double PARK_FORWARD = 12;    // untaught: inches from start, + = away from the wall
    public static double PARK_SIDE = 24;       // untaught: inches from start, + = robot's left (red)
    public static double MIN_OFF_WALL = 3;     // taught: always get at least this far off the wall first
    public static double DRIVE_MAX_S = 8;

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        setup();
        double[] park = AutoSpots.park(alliance());
        List<double[]> via = AutoSpots.points(AutoSpots.parkRoute(alliance()));
        boolean taught = AutoSpots.isSet(park);

        while (opModeInInit()) {
            follower.update();
            telemetry.addData("Auto", "SAFE PARK " + alliance());
            telemetry.addData("Start", "CORNER start spot, touching the wall, not in the LOADING ZONE");
            telemetry.addData("Park at", taught ? AutoSpots.text(park)
                    : "not taught - using settingsMenu numbers (run setupPark" + alliance().title() + ")");
            telemetry.addData("Path points", "%d", via.size());
            showInitChecks();
            telemetry.update();
        }
        if (isStopRequested()) return;
        startTimer();

        waitSeconds(START_DELAY_S);
        if (taught && !via.isEmpty()) {
            step("1/1 to the LOADING ZONE via path points");
            goTo("PARK spot", park, via, DRIVE_MAX_S);
        } else if (taught) {
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
