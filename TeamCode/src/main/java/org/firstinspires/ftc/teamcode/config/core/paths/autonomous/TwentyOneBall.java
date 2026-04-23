package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TwentyOneBall {


    // ---------------- RED POSES ----------------

    public static final Pose startPose = new Pose(40.3, 58.4, 1.557);

    private static final Pose shootPose1 = new Pose(15, 17.3, -0.219); //initial
    //public static final Pose shootPose2 = new Pose(18.6, 8.5, Math.toRadians(-.4)); //before gate intakes
    public static final Pose shootPose2 = new Pose(8.7, -2, Math.toRadians(-.087)); //before gate intakes
    public static final Pose shootPose3 = new Pose(13.4, 14.18, -.71); //after first spike
    public static final Pose shootPoseThirdPickup = new Pose(13, 38, -1.34);

    public static final Pose shootPose4 = new Pose(11, 10, startPose.getHeading());

    private static final Pose moveToShoot = new Pose(30, 0, Math.toRadians(0));
    private static final Pose secondSpikeStrafe = new Pose(20, -12, 0);
    private static final Pose secondSpike = new Pose(53, -12, 0);
    private static final Pose openGate = new Pose(51, -9, 0);
    private static final Pose firstSpikeStrafe = new Pose(20, 10.42, 0);


    private static final Pose firstSpike = new Pose(49.6, 10.42, 0);
    public static final Pose gateKlutch = new Pose(53, 9, 0);

    public static final Pose strafeGate = new Pose(49.2, -14, .7);
    public static final Pose gate = new Pose(58.5, -14.5, .641);

    private static final Pose thirdSpikeStrafe = new Pose(17, -36, 0);

    private static final Pose thirdSpike = new Pose(55, -36, 0);
    private static final Pose move = new Pose(12, 50, shootPose3.getHeading());

    // ---------------- BLUE POSES ----------------

    public static final Pose startPoseBlue = new Pose(-40.7, 60.6, 1.581);//convertToBlue(startPose);

    private static final Pose shootPoseBlue = convertToBlue(shootPose1);
    public static final Pose shootPose2Blue = convertToBlue(shootPose2);
    public static final Pose shootPose3Blue = shootPose2Blue;
    public static final Pose shootPose4Blue = convertToBlue(shootPose4);
    public static final Pose shootPoseThirdPickupBlue = convertToBlue(shootPoseThirdPickup);

    private static final Pose moveToShootBlue = convertToBlue(moveToShoot);
    private static final Pose strafe1Blue = convertToBlue(secondSpikeStrafe);
    private static final Pose pickup2Blue = convertToBlue(secondSpike);
    private static final Pose openGateBlue = convertToBlue(openGate);

    private static final Pose pickup1Blue = convertToBlue(firstSpike);
    public static final Pose gateKlutchBlue = convertToBlue(gateKlutch);

    public static final Pose strafeGateBlue = convertToBlue(strafeGate);
    public static final Pose gateBlue = convertToBlue(gate);

    private static final Pose strafe2Blue = convertToBlue(thirdSpikeStrafe);
    private static final Pose pickup3Blue = convertToBlue(thirdSpike);
    private static final Pose moveBlue = convertToBlue(move);

    // ---------------- RED PATHS (UNCHANGED) ----------------

    public static PathChain shoot1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose4))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose4.getHeading())
                .build();
    }


    public static PathChain secondSpike(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose4, secondSpikeStrafe, secondSpike))//, openGate))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(secondSpike, shootPose2))
                .setConstantHeadingInterpolation(shootPose2.getHeading())
                .build();
    }

    public static PathChain gatePickup(Follower f) {
        return f.pathBuilder()
                //.addPath(new BezierCurve(shootPose2, strafeGate, gate))
                .addPath(new BezierLine(shootPose2, gate))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gate.getHeading())
                .build();
    }

    public static PathChain shootGate(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, shootPose2))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPose2.getHeading())
                //.addPath(new BezierCurve(gate, moveToShoot, shootPose2))
                /*.setReversed()
                .setTangentHeadingInterpolation()
                .setReversed() */
                .build();
    }

    public static PathChain pickupFirstSpike(Follower f) {
        return f.pathBuilder()
                //.addPath(new BezierLine(shootPose2, firstSpike))
                .addPath(new BezierCurve(shootPose2, firstSpikeStrafe, firstSpike))
                .setTangentHeadingInterpolation()
                .build();
    }


    public static PathChain shoot3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(firstSpike, shootPose3))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain thirdSpike (Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose3, thirdSpikeStrafe, thirdSpike))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot4(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(thirdSpike, shootPoseThirdPickup))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    // ---------------- BLUE PATHS (MIRRORED) ----------------

    public static PathChain shoot1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shootPose4Blue))
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