package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class TwelveBall {
    //Red Poses
    public static final Pose startPose = new Pose(51.2, -50.2, -0.83);

    private static final Pose shootPose = new Pose(15.5, -22, -1.57);
    //private static final Pose shootPose2 = new Pose(-0.16, -7.15, -1.567);
    private static final Pose pickup1 = new Pose(14.6, -48.5, -1.567);
    private static final Pose strafeGate = new Pose(6.57, -50, -1.554);
    private static final Pose gate = new Pose(-6, -51.7, -1.56);
    private static final Pose strafe1 = new Pose(-9.25, -25, -1.58);

    private static final Pose pickup2 = new Pose(-9.11, -46, -1.56);

    private static final Pose strafe2 = new Pose(-32.38, -25, -1.57);
    private static final Pose pickup3= new Pose(-32, -49, -1.59);
    private static final Pose strafe3 = new Pose(-54, -15, -1.567 - Math.toRadians(45));
    private static final Pose strafe35 = new Pose(-57, -54, -1.567 - Math.toRadians(45));
    private static final Pose strafe36 = new Pose(-53, -53, -1.567 - Math.toRadians(80));


    private static final Pose pickup4 = new Pose(-57, -57, -1.567- Math.toRadians(80));
    private static final Pose strafe4 = new Pose(-30, -40, -1.58);
    private static final Pose pickup5 = new Pose(-30, -40, -1.58);
    private static final Pose move = new Pose(5, -29.24, -1.588);
    private static final Pose startPoseBlue = new Pose(40.19, 56.96, 0.03);

    private static final Pose shootPoseBlue = new Pose(-1.7, 0, 1.6);
    private static final Pose pickup1Blue = new Pose(-1.7, 42.5, 1.6);
    private static final Pose strafeGateBlue = new Pose(2, 40, 1.588);
    private static final Pose gateBlue = new Pose(2, 42.5, 1.588);

    private static final Pose strafe1Blue = new Pose(-22.7, 8.5, 1.58);
    private static final Pose pickup2Blue = new Pose(-22.7, 44, 1.58);
    private static final Pose strafe2Blue = new Pose(-47, 8.5, 1.58);
    private static final Pose pickup3Blue = new Pose(-47, 44, 1.58);
    private static final Pose moveBlue = new Pose(-5, 20, 1.58);







    public static PathChain shoot1(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup1(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1.getHeading())
                .build();

        return chain;

    }
    public static PathChain gate(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(pickup1, strafeGate))
                .setLinearHeadingInterpolation(pickup1.getHeading(), strafeGate.getHeading())
                .addPath(new BezierCurve(strafeGate, gate))
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot2(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(gate, shootPose))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }

    public static PathChain shoot215(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(pickup1, shootPose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe1(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe1.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup2(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(strafe1, pickup2))
                .setLinearHeadingInterpolation(strafe1.getHeading(), pickup2.getHeading())
                .build();

        return chain;

    }

    public static PathChain gate15(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(pickup2, gate))
                .setLinearHeadingInterpolation(pickup2.getHeading(), gate.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot3(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(pickup2, shootPose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot315(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(gate, shootPose))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe2(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe2.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup3(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(strafe2, pickup3))
                .setLinearHeadingInterpolation(strafe2.getHeading(), pickup3.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot4(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(pickup3, shootPose))
                .setLinearHeadingInterpolation(pickup3.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup4(Follower f) {
        PathChain chain = f.pathBuilder()

                .addPath(new BezierLine(shootPose, strafe3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe3.getHeading())
                .addPath(new BezierLine(strafe3, strafe35))
                .setLinearHeadingInterpolation(strafe3.getHeading(), strafe35.getHeading())
                .addPath(new BezierLine(strafe35, strafe36))
                .setLinearHeadingInterpolation(strafe35.getHeading(), strafe36.getHeading())
                .addPath(new BezierLine(strafe36, pickup4))
                .setLinearHeadingInterpolation(strafe36.getHeading(), pickup4.getHeading())
                .build();

        return chain;

    }

    public static PathChain shoot5(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(pickup4, shootPose))
                .setLinearHeadingInterpolation(pickup4.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup5(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe4.getHeading())
                .addPath(new BezierLine(strafe4, pickup5))
                .setLinearHeadingInterpolation(strafe4.getHeading(), pickup5.getHeading())
                .build();

        return chain;

    }

    public static PathChain shoot6(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(pickup5, shootPose))
                .setLinearHeadingInterpolation(pickup5.getHeading(), shootPose.getHeading())
                .build();

        return chain;

    }
    public static PathChain move(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPose, move))
                .setLinearHeadingInterpolation(shootPose.getHeading(), move.getHeading())
                .build();

        return chain;

    }

    //Blue Stuff

    public static PathChain shoot1Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shootPoseBlue))
                .setLinearHeadingInterpolation((startPoseBlue).getHeading(), shootPoseBlue.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup1Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, pickup1Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), pickup1Blue.getHeading())
                .build();

        return chain;

    }
    public static PathChain gateBlue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(pickup1Blue, strafeGateBlue))
                .setLinearHeadingInterpolation(pickup1Blue.getHeading(), gateBlue.getHeading())
                .addPath(new BezierCurve(strafeGateBlue, gateBlue))
                .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot2Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(gateBlue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup1Blue.getHeading(), shootPoseBlue.getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe1Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, strafe1Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), strafe1Blue.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup2Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(strafe1Blue, pickup2Blue))
                .setLinearHeadingInterpolation(strafe1Blue.getHeading(), pickup2Blue.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot3Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(pickup2Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup2Blue.getHeading(), shootPoseBlue.getHeading())
                .build();

        return chain;

    }
    public static PathChain strafe2Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, strafe2Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), strafe2Blue.getHeading())
                .build();

        return chain;

    }
    public static PathChain pickup3Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierCurve(strafe2Blue, pickup3Blue))
                .setLinearHeadingInterpolation(strafe2Blue.getHeading(), pickup3Blue.getHeading())
                .build();

        return chain;

    }
    public static PathChain shoot4Blue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(pickup3Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup3Blue.getHeading(), shootPoseBlue.getHeading())
                .build();

        return chain;

    }
    public static PathChain moveBlue(Follower f) {
        PathChain chain = f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, moveBlue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), moveBlue.getHeading())
                .build();

        return chain;

    }

    public static Pose convertToBlue(Pose p) {
        return new Pose(p.getX(), -p.getY(), -p.getHeading());
    }

}