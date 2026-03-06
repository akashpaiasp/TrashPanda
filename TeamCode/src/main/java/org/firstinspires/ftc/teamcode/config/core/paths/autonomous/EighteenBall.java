package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class EighteenBall {

    private static double shootConstraint = .99;
    private static double tConstraint = 100;
    private static double braking = 1;
    private static double velConstraint = 0.00000001;

    // ---------------- RED POSES ----------------

    public static final Pose startPose = new Pose(38.7, 60.4, 1.5721);

    private static final Pose shootPose = new Pose(15.5, 12, 0);
    public static final Pose shootPose2 = new Pose(14, 5, Math.toRadians(-15));
    public static final Pose shootPoseThirdPickup = new Pose(12, 36, -.94);

    public static final Pose shootPose4 = new Pose(11, 10, startPose.getHeading());
    public static final Pose shootPose3 = shootPose2;

    private static final Pose moveToShoot = new Pose(30, 0, Math.toRadians(0));
    private static final Pose strafe1 = new Pose(30, -11, 0);
    private static final Pose pickup2 = new Pose(51, -11, 0);
    private static final Pose openGate = new Pose(54.5, -7, 0);

    private static final Pose pickup1 = new Pose(53.16767, 12, 0);
    public static final Pose shoot1Klutch = new Pose(20, 15, Math.toRadians(-120));
    private static final Pose pickup1Klutch = new Pose(45, 13, 0);
    public static final Pose gateKlutch = new Pose(53, 9, 0);

    public static final Pose strafeGate = new Pose(40, -10, 0.59);
    public static final Pose gate = new Pose(59, -11.5, .515);

    private static final Pose strafe2 = new Pose(20, -33.5, 0);
    private static final Pose pickup3 = new Pose(53, -33.5, 0);
    private static final Pose move = new Pose(12, 50, shootPose3.getHeading());

    // ---------------- BLUE POSES ----------------

    public static final Pose startPoseBlue = new Pose(-40.7, 60.6, 1.581);//convertToBlue(startPose);

    private static final Pose shootPoseBlue = convertToBlue(shootPose);
    public static final Pose shootPose2Blue = convertToBlue(shootPose2);
    public static final Pose shootPose3Blue = shootPose2Blue;
    public static final Pose shootPose4Blue = convertToBlue(shootPose4);
    public static final Pose shootPoseThirdPickupBlue = convertToBlue(shootPoseThirdPickup);

    private static final Pose moveToShootBlue = convertToBlue(moveToShoot);
    private static final Pose strafe1Blue = convertToBlue(strafe1);
    private static final Pose pickup2Blue = convertToBlue(pickup2);
    private static final Pose openGateBlue = convertToBlue(openGate);

    private static final Pose pickup1Blue = convertToBlue(pickup1);
    public static final Pose shoot1KlutchBlue = convertToBlue(shoot1Klutch);
    private static final Pose pickup1KlutchBlue = convertToBlue(pickup1Klutch);
    public static final Pose gateKlutchBlue = convertToBlue(gateKlutch);

    public static final Pose strafeGateBlue = convertToBlue(strafeGate);
    public static final Pose gateBlue = convertToBlue(gate);

    private static final Pose strafe2Blue = convertToBlue(strafe2);
    private static final Pose pickup3Blue = convertToBlue(pickup3);
    private static final Pose moveBlue = convertToBlue(move);

    // ---------------- RED PATHS (UNCHANGED) ----------------

    public static PathChain shoot1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose4))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose4.getHeading())
                .build();
    }

    public static PathChain shoot1Klutch(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Klutch))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose4.getHeading())
                .build();
    }

    public static PathChain pickup1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose2, strafe1, pickup2, openGate))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup2, shootPose2))
                .setConstantHeadingInterpolation(shootPose2.getHeading())
                .build();
    }

    public static PathChain gatePickup(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2, strafeGate))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), strafeGate.getHeading())
                .addPath(new BezierLine(strafeGate, gate))
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();
    }

    public static PathChain gatePickup2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2, strafeGate))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), strafeGate.getHeading())
                .addPath(new BezierLine(strafeGate, gate))
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();
    }

    public static PathChain shootGate(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(gate, moveToShoot, shootPose2))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain shootGate2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(gate, moveToShoot, shootPose2))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain pickup2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup1))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    public static PathChain pickup1Klutch(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickup1Klutch, gateKlutch))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1, shootPose2))
                .setConstantHeadingInterpolation(shootPose2.getHeading())
                .build();
    }

    public static PathChain shoot3Klutch(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1, shootPoseThirdPickup))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shootPoseThirdPickup.getHeading())
                .build();
    }

    public static PathChain pickup3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose2, strafe2, pickup3))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot4(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup3, shootPoseThirdPickup))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain move(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose3, move))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), move.getHeading())
                .build();
    }

    // ---------------- BLUE PATHS (MIRRORED) ----------------

    public static PathChain shoot1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shootPose4Blue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), shootPose4Blue.getHeading())
                .build();
    }

    public static PathChain shoot1KlutchBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shoot1KlutchBlue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), shootPose4Blue.getHeading())
                .build();
    }

    public static PathChain pickup1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose2Blue, strafe1Blue, pickup2Blue, openGateBlue))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup2Blue, shootPose2Blue))
                .setConstantHeadingInterpolation(shootPose2Blue.getHeading())
                .build();
    }

    public static PathChain gatePickupBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, strafeGateBlue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), strafeGateBlue.getHeading())
                .addPath(new BezierLine(strafeGateBlue, gateBlue))
                .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                .build();
    }

    public static PathChain gatePickup2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, strafeGateBlue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), strafeGateBlue.getHeading())
                .addPath(new BezierLine(strafeGateBlue, gateBlue))
                .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                .build();
    }

    public static PathChain shootGateBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(gateBlue, moveToShootBlue, shootPose2Blue))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain shootGate2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(gateBlue, moveToShootBlue, shootPose2Blue))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain pickup2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, pickup1Blue))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    public static PathChain pickup1KlutchBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPoseBlue, pickup1KlutchBlue, gateKlutchBlue))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot3Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1Blue, shootPose2Blue))
                .setConstantHeadingInterpolation(shootPose2Blue.getHeading())
                .build();
    }

    public static PathChain shoot3KlutchBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1Blue, shootPoseThirdPickupBlue))
                .setLinearHeadingInterpolation(pickup1Blue.getHeading(), shootPoseThirdPickupBlue.getHeading())
                .build();
    }

    public static PathChain pickup3Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose2Blue, strafe2Blue, pickup3Blue))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot4Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup3Blue, shootPoseThirdPickupBlue))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain moveBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose3Blue, moveBlue))
                .setLinearHeadingInterpolation(shootPose3Blue.getHeading(), moveBlue.getHeading())
                .build();
    }

    // ---------------- CONVERSION ----------------

    public static Pose convertToBlue(Pose p) {
        return new Pose(-p.getX(), p.getY(), Math.PI - p.getHeading());
    }
}