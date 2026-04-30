package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TwentyOneBall {


    // ---------------- RED POSES ----------------

    public static final Pose startPose = new Pose(38, 58.7, Math.PI - 1.569);

    private static final Pose shootPose1 = new Pose(15, 17.3, -0.219); //initial
    //public static final Pose shootPose2 = new Pose(18.6, 8.5, Math.toRadians(-.4)); //before gate intakes
    public static final Pose shootPose2 = new Pose(8.7, -2, Math.toRadians(-.087)); //before gate intakes
    public static final Pose shootPose3 = new Pose(13.4, 14.18, -.71); //after first spike
    public static final Pose shootPoseThirdPickup = new Pose(13, 38, -1.34);

    public static final Pose shootPose4 = new Pose(11, 10, 0.7525);

    private static final Pose moveToShoot = new Pose(30, 0, Math.toRadians(0));
    private static final Pose secondSpikeStrafe = new Pose(20, -12, 0);
    private static final Pose secondSpike = new Pose(53, -12, 0);
    private static final Pose openGate = new Pose(51, -9, 0);
    private static final Pose firstSpikeStrafe = new Pose(20, 10.42, 0);


    private static final Pose firstSpike = new Pose(49.6, 10.42, 0);
    public static final Pose gateKlutch = new Pose(53, 9, 0);

    public static final Pose strafeGate = new Pose(49.2, -14, .7);
    public static final Pose gate = new Pose(57.5, -12.38, Math.PI - 2.624 );

    private static final Pose thirdSpikeStrafe = new Pose(17, -36, 0);

    private static final Pose thirdSpike = new Pose(55, -36, 0);
    private static final Pose move = new Pose(12, 50, shootPose3.getHeading());

    // ---------------- BLUE POSES ----------------

    // ---------------- BLUE POSES (CORRECT MIRROR OF RED) ----------------

    public static final Pose startPoseBlue = convertToBlue(startPose);

    private static final Pose shootPose1Blue = convertToBlue(shootPose1);
    public static final Pose shootPose2Blue = convertToBlue(shootPose2);
    public static final Pose shootPose3Blue = convertToBlue(shootPose3);
    public static final Pose shootPose4Blue = convertToBlue(shootPose4);
    public static final Pose shootPoseThirdPickupBlue = convertToBlue(shootPoseThirdPickup);

    private static final Pose moveToShootBlue = convertToBlue(moveToShoot);

    private static final Pose secondSpikeStrafeBlue = convertToBlue(secondSpikeStrafe);
    private static final Pose secondSpikeBlue = convertToBlue(secondSpike);
    private static final Pose openGateBlue = convertToBlue(openGate);

    private static final Pose firstSpikeStrafeBlue = convertToBlue(firstSpikeStrafe);
    private static final Pose firstSpikeBlue = convertToBlue(firstSpike);

    public static final Pose gateKlutchBlue = convertToBlue(gateKlutch);

    public static final Pose strafeGateBlue = convertToBlue(strafeGate);
    public static final Pose gateBlue = convertToBlue(gate);

    private static final Pose thirdSpikeStrafeBlue = convertToBlue(thirdSpikeStrafe);
    private static final Pose thirdSpikeBlue = convertToBlue(thirdSpike);

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

    public static PathChain shoot3NoThird(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(firstSpike, shootPoseThirdPickup))
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

    public static PathChain shoot4Alliance(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, shootPoseThirdPickup))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    // ---------------- BLUE PATHS (MIRRORED) ----------------

    // ---------------- BLUE PATHS (MATCH RED EXACTLY) ----------------

    // ---------------- BLUE PATHS (MATCH RED STRUCTURE EXACTLY) ----------------

    public static PathChain shoot1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shootPose4Blue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), shootPose4Blue.getHeading())
                .build();
    }

    public static PathChain secondSpikeBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose4Blue, secondSpikeStrafeBlue, secondSpikeBlue))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(secondSpikeBlue, shootPose2Blue))
                .setConstantHeadingInterpolation(shootPose2Blue.getHeading())
                .build();
    }

    public static PathChain gatePickupBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, gateBlue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), gateBlue.getHeading())
                .build();
    }

    public static PathChain shootGateBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gateBlue, shootPose2Blue))
                .setLinearHeadingInterpolation(gateBlue.getHeading(), shootPose2Blue.getHeading())
                .build();
    }

    public static PathChain pickupFirstSpikeBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose2Blue, firstSpikeStrafeBlue, firstSpikeBlue))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot3Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(firstSpikeBlue, shootPose3Blue))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
    public static PathChain shoot3NoThirdBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(firstSpikeBlue, shootPoseThirdPickupBlue))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public static PathChain thirdSpikeBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose3Blue, thirdSpikeStrafeBlue, thirdSpikeBlue))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shoot4Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(thirdSpikeBlue, shootPoseThirdPickupBlue))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
    public static PathChain shoot4AllianceBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gateBlue, shootPoseThirdPickupBlue))
                .setReversed()
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    // ---------------- CONVERSION ----------------

    public static Pose convertToBlue(Pose p) {
        return new Pose(-p.getX(), p.getY(), Math.PI - p.getHeading());
    }
}