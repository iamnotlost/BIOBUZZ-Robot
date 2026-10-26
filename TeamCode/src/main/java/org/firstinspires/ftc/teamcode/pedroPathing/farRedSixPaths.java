package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class farRedSixPaths {
    public static Pose startPose = new Pose(82.000, -2.000, Math.toRadians(90));

    public PathChain Path1;
    public PathChain PreRotate2;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain PreRotate5;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    public farRedSixPaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(82.000, -2.000),
                        new Pose(84.000, 10.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68.5))
                .build();

        PreRotate2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 10.000),
                        new Pose(84.010, 20.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(68.5), Math.toRadians(-35))
                .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 20.000),
                        new Pose(142.000, 20.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(-35))
                .build();
        Path3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(142.000, 20.000),
                        new Pose(136.000, 8.000),
                        new Pose(144.000, 3.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(-90))
                .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(144.000, 3.000),
                        new Pose(130.000, 22.000),
                        new Pose(84.000, 10.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(68.5))
                .build();

        PreRotate5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 10.000),
                        new Pose(84.010, 20.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(68.5), Math.toRadians(-35))
                .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 20.000),
                        new Pose(142.000, 20.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(-35))
                .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(142.000, 20.000),
                        new Pose(136.000, 8.000),
                        new Pose(144.000, 3.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(-90))
                .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(144.000, 3.000),
                        new Pose(130.000, 22.000),
                        new Pose(84.000, 10.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(68.5))
                 .build();
        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(84.000, 10.000),
                        new Pose(114.000, 28.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(68.5), Math.toRadians(0))
                .build();
    }
}
