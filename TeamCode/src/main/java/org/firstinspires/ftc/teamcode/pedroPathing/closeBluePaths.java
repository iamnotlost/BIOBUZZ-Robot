package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class closeBluePaths {
    public static Pose startPose = new Pose(24.000, 140.000, Math.toRadians(90));

    public static PathChain vBluePath1(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(24.000, 140.000),
                        new Pose(37.916, 123.032),
                        new Pose(36.000, 108.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
    }

    public static PathChain vBluePath2(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(36.000, 108.000),
                        new Pose(48.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    public static PathChain vBluePath3(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(48.000, 84.000),
                        new Pose(19.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain vBluePath4(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(19.000, 84.000),
                        new Pose(44.000, 75.000),
                        new Pose(18.000, 70.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain vBluePath5(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(18.000, 70.000),
                        new Pose(36.000, 108.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    public static PathChain vBluePath6(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(36.000, 108.000),
                        new Pose(82.000, 48.000),
                        new Pose(17.000, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    public static PathChain vBluePath7(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(17.000, 60.000),
                        new Pose(40.000, 60.000),
                        new Pose(36.000, 108.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    public static PathChain vBluePath8(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(36.000, 108.000),
                        new Pose(83.994, 24.006),
                        new Pose(17.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    public static PathChain vBluePath9(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(17.000, 36.000),
                        new Pose(40.000, 36.000),
                        new Pose(36.000, 108.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    public static PathChain vBluePath10(Follower follower) {
        PathBuilder builder = new PathBuilder(follower);
        return builder
                .addPath(new BezierLine(new Pose(36.000, 108.000), new Pose(36.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }
}
