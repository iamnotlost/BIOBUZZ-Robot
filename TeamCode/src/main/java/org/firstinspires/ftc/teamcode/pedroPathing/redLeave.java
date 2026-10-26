package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class redLeave {
    public static PathChain redLeavePath(Follower follower) {
        PathBuilder builder = new PathBuilder(follower);
        return builder
                .addPath(
                        new BezierLine(new Pose(0, 0), new Pose(30, 0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
}