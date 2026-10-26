package org.firstinspires.ftc.teamcode;

// LEAVE (3) + AUTO PARK (5): drive off the wall, then into our LOADING ZONE.
// Two robots doing this gives 16 pts, which is the SWARM RP threshold.
//
// The PARK offsets are guesses: measure from your start spot to the LOADING ZONE on a real field
// and set them. PARK_SIDE is for red; blue is mirrored automatically.
public abstract class baseParkAuto extends baseAuto {
    public static double START_DELAY_S  = 0;
    public static double LEAVE_DISTANCE = 12;   // inches straight out from the wall
    public static double PARK_FORWARD   = 12;   // inches from start, + = away from the wall
    public static double PARK_SIDE      = 24;   // inches from start, + = robot's left (red)

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        setup();
        telemetry.addData("Alliance", alliance());
        telemetry.update();
        waitForStart();
        startTimer();

        waitSeconds(START_DELAY_S);
        runPath(lineTo(LEAVE_DISTANCE, 0));
        runPath(lineTo(PARK_FORWARD, alliance().sign * PARK_SIDE));

        stopAll();
    }
}
