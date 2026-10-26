package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class farBlueNinePaths {
    public static Pose startPose = new Pose(55.000, 9.000, Math.toRadians(90));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain PreRotate5;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public farBlueNinePaths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55.000, 9.000),
                        new Pose(60.000, 12.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113.5))
                .build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(60.000, 12.000),
                        new Pose(60.000, 25.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(113.5), Math.toRadians(180))
                .build();
        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(60.000, 25.000),
                        new Pose(20.000, 25.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20.000, 25.000),
                        new Pose(60.000, 12.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113.5))
                .build();

        PreRotate5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(60.000, 12.000),
                        new Pose(60.010, 12.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(113.5), Math.toRadians(170))
                .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(60.000, 12.000),
                        new Pose(40.000, 30.000),
                        new Pose(23.000, 5.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(270))
                .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(23.000, 5.000),
                        new Pose(60.000, 12.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(113.5))
                .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(60.000, 12.000),
                        new Pose(30.000, 20.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(113.5), Math.toRadians(180)).build();
    }
}
