package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class EighteenBall {
    private static double shootConstraint = .99;
    private static double tConstraint = 100;
    private static double braking = 1;
    private static double velConstraint = 0.00000001;
    // Red Poses
    public static final Pose startPose = new Pose(50, 48.6, .73);

    private static final Pose shootPose = new Pose(12, 12, 0);
    public static final Pose shootPose2 = new Pose(16, 5, Math.toRadians(-15));
    public static final Pose shootPoseThirdPickup = new Pose(16, 10, -.94);

    public static final Pose shootPose4 = new Pose(16, 5, startPose.getHeading());
    public static final Pose shootPose3 = new Pose(11, 5, Math.toRadians(-15));
    private static final Pose moveToShoot = new Pose(24, 0, Math.toRadians(0));
    private static final Pose strafe1 = new Pose(30, -11.7, 0);
    private static final Pose pickup2 = new Pose(54, -11.7, 0); //second spike mark
    private static final Pose pickup1 = new Pose(50, 12, 0); //first spike mark

    public static final Pose strafeGate = new Pose(40, -10, 0.59);
    public static final Pose gate = new Pose(56.5, -14, .449);
    public static final Pose gateBlue = convertToBlue(gate);//new Pose(-59, -10, 2.55);
    private static final Pose strafe2 = new Pose(20, -36, 0);
    private static final Pose pickup3 = new Pose(53, -36, 0);
    private static final Pose move = new Pose(24, 0, shootPose3.getHeading());

    // Blue Poses
    public static final Pose startPoseBlue = convertToBlue(startPose);//new Pose(49.6, 49.9, 0.77);

    private static final Pose shootPoseBlue = convertToBlue(shootPose);//new Pose(12, 12, 0);
    public static final Pose shootPose2Blue = convertToBlue(shootPose2);//new Pose(16, 5, Math.toRadians(-15));
    private static final Pose moveToShootBlue = convertToBlue(moveToShoot);//new Pose(24, 0, Math.toRadians(0));
    private static final Pose strafe1Blue = convertToBlue(strafe1);//new Pose(30, -11.7, 0);
    private static final Pose pickup2Blue = convertToBlue(pickup2);//new Pose(50, -11.7, 0); //second spike mark
    private static final Pose pickup1Blue = convertToBlue(pickup1);//new Pose(50, 12, 0); //first spike mark
    public static final Pose strafeGateBlue = convertToBlue(strafeGate);//new Pose(40, -10, 0.59);
    //public static final Pose gateBlue = convertToBlue(gate);//new Pose(60, -12.5, 0.6487);
    private static final Pose strafe2Blue = convertToBlue(strafe2);//new Pose(30, -36, 0);
    private static final Pose pickup3Blue = convertToBlue(pickup3);//new Pose(53, -36, 0);
    private static final Pose moveBlue = convertToBlue(move);//new Pose(24, 0, 0.77);


    public static PathChain shoot1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose4))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose4.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain strafe1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2, strafe1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), strafe1.getHeading())
                .build();
    }

    public static PathChain pickup1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(strafe1, pickup2))
                .setLinearHeadingInterpolation(strafe1.getHeading(), pickup2.getHeading())
                .build();
    }


    public static PathChain shoot2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup2, shootPose2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shootPose2.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain gatePickup(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2, strafeGate))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), strafeGate.getHeading())
                .addPath(new BezierLine(strafeGate, gate))
                .setBrakingStart(.8)
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();
    }
    public static PathChain gatePickup2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2, strafeGate))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), strafeGate.getHeading())
                .addPath(new BezierLine(strafeGate, gate))
                .setBrakingStart(.8)
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();
    }

    public static PathChain shootGate(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, moveToShoot))
                .setLinearHeadingInterpolation(gate.getHeading(), moveToShoot.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .addPath(new BezierLine(moveToShoot, shootPose2))
                .setLinearHeadingInterpolation(moveToShoot.getHeading(), shootPose2.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain shootGate2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, moveToShoot))
                .setLinearHeadingInterpolation(gate.getHeading(), moveToShoot.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .addPath(new BezierLine(moveToShoot, shootPose))
                .setLinearHeadingInterpolation(moveToShoot.getHeading(), shootPose.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain pickup2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup1))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    public static PathChain shoot3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1, shootPose2))
                .setConstantHeadingInterpolation(shootPose2.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain strafe2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2, strafe2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), strafe2.getHeading())
                //.setLinearHeadingInterpolation(shootPose2.getHeading(), strafe2.getHeading())
                .build();
    }

    public static PathChain pickup3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(strafe2, pickup3))
                .setConstantHeadingInterpolation(pickup3.getHeading())
                .build();
    }

    public static PathChain shoot4(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup3, shootPoseThirdPickup))
                .setConstantHeadingInterpolation(shootPoseThirdPickup.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain move(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose3, move))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), move.getHeading())
                .build();
    }


    // -------------------- BLUE PATHS (FULL SET) --------------------
    public static PathChain shoot1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shootPose2Blue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), shootPose2Blue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain strafe1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, strafe1Blue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), strafe1Blue.getHeading())
                .build();
    }

    public static PathChain pickup1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(strafe1Blue, pickup2Blue))
                .setLinearHeadingInterpolation(strafe1Blue.getHeading(), pickup2Blue.getHeading())
                .build();
    }

    public static PathChain shoot2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup2Blue, shootPose2Blue))
                .setLinearHeadingInterpolation(pickup2Blue.getHeading(), shootPose2Blue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain gatePickupBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, strafeGateBlue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), strafeGateBlue.getHeading())
                .addPath(new BezierLine(strafeGateBlue, gateBlue))
                .setBrakingStart(.8)
                .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                .build();
    }

    public static PathChain gatePickup2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, strafeGateBlue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), strafeGateBlue.getHeading())
                .addPath(new BezierLine(strafeGateBlue, gateBlue))
                .setBrakingStart(.8)
                .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                .build();
    }

    public static PathChain shootGateBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gateBlue, moveToShootBlue))
                .setLinearHeadingInterpolation(gateBlue.getHeading(), moveToShootBlue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .addPath(new BezierLine(moveToShootBlue, shootPose2Blue))
                .setLinearHeadingInterpolation(moveToShootBlue.getHeading(), shootPose2Blue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain shootGate2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gateBlue, moveToShootBlue))
                .setLinearHeadingInterpolation(gateBlue.getHeading(), moveToShootBlue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .addPath(new BezierLine(moveToShootBlue, shootPoseBlue))
                .setLinearHeadingInterpolation(moveToShootBlue.getHeading(), shootPoseBlue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain pickup2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, pickup1Blue))
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    public static PathChain shoot3Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1Blue, shootPose2Blue))
                .setLinearHeadingInterpolation(pickup1Blue.getHeading(), shootPose2Blue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain strafe2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, strafe2Blue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), strafe2Blue.getHeading())
                .build();
    }

    public static PathChain pickup3Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(strafe2Blue, pickup3Blue))
                .setLinearHeadingInterpolation(strafe2Blue.getHeading(), pickup3Blue.getHeading())
                .build();
    }

    public static PathChain shoot4Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup3Blue, shootPose2Blue))
                .setLinearHeadingInterpolation(pickup3Blue.getHeading(), shootPose2Blue.getHeading())
                .setTValueConstraint(shootConstraint)
                .setTimeoutConstraint(tConstraint)
                .setBrakingStrength(braking)
                .setVelocityConstraint(velConstraint)
                .build();
    }

    public static PathChain moveBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose2Blue, moveBlue))
                .setLinearHeadingInterpolation(shootPose2Blue.getHeading(), moveBlue.getHeading())
                .build();
    }



    // Convert RED pose to BLUE field pose
    public static Pose convertToBlue(Pose p) {
        return new Pose(-p.getX(), p.getY(),  Math.PI - p.getHeading());
    }
}