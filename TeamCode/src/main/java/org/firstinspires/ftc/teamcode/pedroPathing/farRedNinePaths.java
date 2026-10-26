package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class farRedNinePaths {
    public static Pose startPose = new Pose(82.000, -2.000, Math.toRadians(90));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain PreRotate5;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public farRedNinePaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(82.000, -2.000),
                        new Pose(84.000, 10.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68.5))
                .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 10.000),
                        new Pose(94.000, 36.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(68.5), Math.toRadians(0))
                .build();
        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(94.000, 36.000),
                        new Pose(135.000, 36.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(135.000, 36.000),
                        new Pose(84.000, 10.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68.5))
                .build();

        PreRotate5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 10.000),
                        new Pose(84.010, 10.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(68.5), Math.toRadians(10))
                .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(84.000, 10.000),
                        new Pose(135.000, 35.000),
                        new Pose(155.000, -0.500)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(-90))
                .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(155.000, -0.500),
                        new Pose(84.000, 10.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(68.5))
                .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 10.000),
                        new Pose(114.000, 28.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(68.5), Math.toRadians(0)).build();
    }
}
