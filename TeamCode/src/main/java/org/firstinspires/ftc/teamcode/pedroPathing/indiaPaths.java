package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class indiaPaths {
    public static Pose startPose = new Pose(105.0, 122.0, Math.toRadians(90));

    public static PathChain vRedPath1(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(105.000, 122.000),
                        new Pose(106.084, 123.032),
                        new Pose(108.000, 108.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
    }

    public static PathChain vRedPath2(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(108.000, 108.000),
                        new Pose(96.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }

    public static PathChain vRedPath3(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(96.000, 84.000),
                        new Pose(132.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain vRedPath4(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(132.000, 84.000),
                        new Pose(100.000, 75.000),
                        new Pose(130.000, 70.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain vRedPath5(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.000, 70.000),
                        new Pose(108.000, 108.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    public static PathChain vRedPath6(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(108.000, 108.000),
                        new Pose(62.000, 48.000),
                        new Pose(130.000, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }
    public static PathChain vRedPath7(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(130.000, 60.000),
                        new Pose(100.000, 75.000),
                        new Pose(130.000, 70.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain vRedPath8(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(130.000, 70.000),
                        new Pose(104.000, 60.000),
                        new Pose(108.000, 108.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    public static PathChain vRedPath9(Follower follower) {
        PathBuilder builder = new PathBuilder(follower);
        return builder
                .addPath(new BezierLine(new Pose(108.000, 108.000), new Pose(118.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }
}
