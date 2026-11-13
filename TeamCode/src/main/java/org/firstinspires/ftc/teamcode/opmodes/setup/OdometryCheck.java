package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * Checks the odometry counts the right way while you drive: driving FORWARD must make "ahead" go up, driving LEFT
 * must make "left" go up, turning LEFT must make "turned" go up. Each score adds up how far the robot moved in the
 * direction the sticks asked for (negative = the odometry counts the wrong way).
 */
class OdometryCheck {
    private static final double CHECK_IN = 6, CHECK_RAD = Math.toRadians(20);

    private Pose last = null;
    private double forward, left, turn;
    private boolean bad, justWentBad;

    void update(Pose p, double fwdCmd, double leftCmd, double turnCmd) {
        justWentBad = false;
        if (last != null) {
            double dx = p.getX() - last.getX(), dy = p.getY() - last.getY(), h = last.getHeading();
            double movedFwd = dx * Math.cos(h) + dy * Math.sin(h);     // in the robot's own directions
            double movedLeft = -dx * Math.sin(h) + dy * Math.cos(h);
            double turned = Angles.wrap(p.getHeading() - h);
            if (Math.abs(fwdCmd) > 0.04 && Math.abs(fwdCmd) > 2 * Math.abs(leftCmd)) forward += Math.signum(fwdCmd) * movedFwd;
            if (Math.abs(leftCmd) > 0.04 && Math.abs(leftCmd) > 2 * Math.abs(fwdCmd)) left += Math.signum(leftCmd) * movedLeft;
            if (Math.abs(turnCmd) > 0.04 && Math.abs(fwdCmd) + Math.abs(leftCmd) < 0.02) turn += Math.signum(turnCmd) * turned;
            boolean wasBad = bad;
            bad = forward < -CHECK_IN || left < -CHECK_IN || turn < -CHECK_RAD;
            justWentBad = bad && !wasBad;
        }
        last = p;
    }

    boolean bad()         { return bad; }
    boolean justWentBad() { return justWentBad; }

    String text() {
        return "forward " + result(forward, CHECK_IN) + ", left " + result(left, CHECK_IN)
                + ", turn left " + result(turn, CHECK_RAD)
                + (bad ? "  !! COUNTS THE WRONG WAY - don't save spots, tell the coder" : "");
    }

    private static String result(double score, double needed) {
        if (score > needed) return "OK";
        if (score < -needed) return "WRONG WAY";
        return "drive to check";
    }
}
