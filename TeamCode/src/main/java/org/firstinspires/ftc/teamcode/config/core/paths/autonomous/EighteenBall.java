package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class EighteenBall {
    private static double shootConstraint = .9;
    // Red Poses
    public static final Pose startPose = new Pose(49.6, 49.9, 0.77);

    private static final Pose shootPose = new Pose(12, 12, 0.77);
    private static final Pose shootPose3 = new Pose(12, 5, Math.toRadians(-15));
    private static final Pose moveToShoot = new Pose(24, 0, Math.toRadians(0));
    private static final Pose shootPose2 = new Pose(18, 6, Math.toRadians(27));

    private static final Pose strafe1 = new Pose(30, -11.7, 0);
    private static final Pose pickup2 = new Pose(50, -11.7, 0); //second spike mark
    private static final Pose pickup1 = new Pose(50, 12, 0); //first spike mark

    private static final Pose strafeGate = new Pose(40, -10, 0.59);
    private static final Pose gate = new Pose(59, -12.5, 0.6487);
    private static final Pose strafe2 = new Pose(30, -36, 0);
    private static final Pose pickup3 = new Pose(50, -36, 0);
    private static final Pose strafe3 = new Pose(-54, -50, -1.567 - Math.toRadians(45));
    private static final Pose strafe35 = new Pose(-57, -54, -1.567 - Math.toRadians(45));
    private static final Pose strafe36 = new Pose(-53, -53, -1.567 - Math.toRadians(90));

    private static final Pose strafeGate18 = new Pose(-.8, -45.1, -.94);
    private static final Pose openGate18 = new Pose(0, -49.7, -1.05);
    private static final Pose moveToIntake18 = new Pose(-21.4, -51.1, -1.05);
    private static final Pose intakeGate18 = new Pose(-20.42, -55.92, -1.05);

    private static final Pose pickup4 = new Pose(-57, -57, -1.567 - Math.toRadians(90));
    private static final Pose strafe4 = new Pose(-30, -40, -1.58);
    private static final Pose pickup5 = new Pose(-30, -40, -1.58);
    private static final Pose move = new Pose(30, 5, 0);

    // Blue Poses
    public static final Pose startPoseBlue = new Pose(51.8, 50.3, 0.81);//convertToBlue(startPose);

    private static final Pose shootPoseBlue = new Pose(13.7, 14.7, 1.55);//convertToBlue(shootPose);
    private static final Pose pickup1Blue = new Pose(14.55, 53, 1.55);//convertToBlue(pickup1);
    private static final Pose strafeGateBlue = new Pose(-13, 46, 1.54);
    private static final Pose strafe1Blue = new Pose(-10.89, 14.44, 1.56);//convertToBlue(strafe1); //12, -22, 1.48
    //pickup second set
    private static final Pose pickup2Blue = new Pose(-10, 60, 1.54);
    //hit the gate
    private static final Pose gateBlue = new Pose(6, 55, 1.55);//convertToBlue(gate);
    private static final Pose strafe2Blue = new Pose(-33.24, 15.29, 1.55);//convertToBlue(strafe2);
    private static final Pose pickup3Blue = new Pose(-34, 58.5, 1.574);//convertToBlue()//convertToBlue(pickup3);
    /*
    private static final Pose strafe3Blue = new Pose(-15.7, 58.4, 2.183);//(-7.77, 27.5, 2.13);//convertToBlue(strafe3);
    private static final Pose strafe35Blue = new Pose(-55.76, 58.66, 2.19);//(-15.7, 58.4, 2.183);//convertToBlue(strafe35);
    private static final Pose strafe36Blue = new Pose(-59, 55, 1.56);

     */
    private static final Pose strafe3Blue = new Pose(-56.7, 50, 2.123);//(-7.77, 27.5, 2.13);//convertToBlue(strafe3);
    private static final Pose strafe35Blue = new Pose(-56.3, 58, 2.36);//(-15.7, 58.4, 2.183);//convertToBlue(strafe35);
    private static final Pose strafe36Blue = new Pose(-61.45, 55, 1.56);

    //private static final Pose pickup4Blue = new Pose(-59, 60, 1.56);//convertToBlue(pickup4);
    public static final Pose pickup4Blue = new Pose(-61.45, 63, 1.56);
    private static final Pose strafe4Blue = convertToBlue(strafe4);
    private static final Pose pickup5Blue = convertToBlue(pickup5);
    private static final Pose moveBlue = new Pose(0, 30, 1.56);



    //18 ball stuff
    private static final Pose strafeGate18Blue = convertToBlue(strafeGate18);
    private static final Pose openGate18Blue = convertToBlue(openGate18);
    private static final Pose moveToIntake18Blue = convertToBlue(moveToIntake18);
    private static final Pose intakeGate18Blue = convertToBlue(intakeGate18);


    public static PathChain shoot1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose3))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose3.getHeading())
                .setTValueConstraint(shootConstraint)
                .build();
    }

    public static PathChain strafe1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose3, strafe1))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), strafe1.getHeading())
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
                .addPath(new BezierLine(pickup2, shootPose3))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shootPose3.getHeading())
                .setTValueConstraint(shootConstraint)
                .build();
    }

    public static PathChain gatePickup(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose3, strafeGate))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), strafeGate.getHeading())
                .addPath(new BezierLine(strafeGate, gate))
                .setBrakingStart(.8)
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();
    }
    public static PathChain gatePickup2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose3, strafeGate))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), strafeGate.getHeading())
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
                .addPath(new BezierLine(moveToShoot, shootPose3))
                .setLinearHeadingInterpolation(moveToShoot.getHeading(), shootPose3.getHeading())
                .setTValueConstraint(shootConstraint)
                .build();
    }

    public static PathChain pickup2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose3, pickup1))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), pickup1.getHeading())
                .build();
    }

    public static PathChain shoot3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(pickup1, shootPose3))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shootPose3.getHeading())
                .setTValueConstraint(shootConstraint)
                .build();
    }

    public static PathChain strafe2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose3, strafe2))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), strafe2.getHeading())
                .build();
    }

    public static PathChain pickup3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(strafe2, pickup3))
                .setLinearHeadingInterpolation(strafe2.getHeading(), pickup3.getHeading())
                .build();
    }

    public static PathChain shoot4(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup3, shootPose3))
                .setLinearHeadingInterpolation(pickup3.getHeading(), shootPose3.getHeading())
                .setTValueConstraint(shootConstraint)
                .build();
    }

    public static PathChain move(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, move))
                .setLinearHeadingInterpolation(shootPose.getHeading(), move.getHeading())
                .build();
    }


    // -------------------- BLUE PATHS (FULL SET) --------------------

    public static PathChain shoot1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shootPoseBlue))
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain pickup1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, pickup1Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), pickup1Blue.getHeading())
                .build();
    }

    public static PathChain gateBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(pickup1Blue, strafeGateBlue))
                .setLinearHeadingInterpolation(pickup1Blue.getHeading(), strafeGateBlue.getHeading())
                .addPath(new BezierCurve(strafeGateBlue, gateBlue))
                .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                .build();
    }

    public static PathChain shoot2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gateBlue, shootPoseBlue))
                .setLinearHeadingInterpolation(gateBlue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain shoot215Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup1Blue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain strafe1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, strafe1Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), strafe1Blue.getHeading())
                .build();
    }

    public static PathChain pickup2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(strafe1Blue, pickup2Blue))
                .setLinearHeadingInterpolation(strafe1Blue.getHeading(), pickup2Blue.getHeading())
                .build();
    }

    public static PathChain gate15Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(pickup2Blue, strafeGateBlue))
                .setLinearHeadingInterpolation(pickup2Blue.getHeading(), strafeGateBlue.getHeading())
                .addPath(new BezierCurve(strafeGateBlue, gateBlue))
                .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                .build();
    }

    public static PathChain gate18Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPoseBlue, strafeGate18Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), strafeGate18Blue.getHeading())
                .addPath(new BezierCurve(strafeGate18Blue, openGate18Blue))
                .setLinearHeadingInterpolation(strafeGate18Blue.getHeading(), openGate18Blue.getHeading())
                .build();
    }

    public static PathChain intakeGate18Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(openGate18Blue, moveToIntake18Blue))
                .setLinearHeadingInterpolation(openGate18Blue.getHeading(), moveToIntake18Blue.getHeading())
                .addPath(new BezierCurve(moveToIntake18Blue, intakeGate18Blue))
                .setLinearHeadingInterpolation(moveToIntake18Blue.getHeading(), intakeGate18Blue.getHeading())
                .build();
    }

    public static PathChain shoot18Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(intakeGate18Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(intakeGate18Blue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain shoot3Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(pickup2Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup2Blue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain shoot315Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(gateBlue, shootPoseBlue))
                .setLinearHeadingInterpolation(gateBlue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain strafe2Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, strafe2Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), strafe2Blue.getHeading())
                .build();
    }

    public static PathChain pickup3Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(strafe2Blue, pickup3Blue))
                .setLinearHeadingInterpolation(strafe2Blue.getHeading(), pickup3Blue.getHeading())
                .build();
    }

    public static PathChain shoot4Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup3Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup3Blue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain pickup4Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, strafe3Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), strafe3Blue.getHeading())
                .addPath(new BezierLine(strafe3Blue, strafe35Blue))
                .setLinearHeadingInterpolation(strafe3Blue.getHeading(), strafe35Blue.getHeading())
                .addPath(new BezierLine(strafe35Blue, strafe36Blue))
                .setLinearHeadingInterpolation(strafe35Blue.getHeading(), strafe36Blue.getHeading())
                .addPath(new BezierLine(strafe36Blue, pickup4Blue))
                .setLinearHeadingInterpolation(strafe36Blue.getHeading(), pickup4Blue.getHeading())
                .build();
    }

    public static PathChain shoot5Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup4Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup4Blue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain pickup5Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, strafe4Blue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), strafe4Blue.getHeading())
                .addPath(new BezierLine(strafe4Blue, pickup5Blue))
                .setLinearHeadingInterpolation(strafe4Blue.getHeading(), pickup5Blue.getHeading())
                .build();
    }

    public static PathChain shoot6Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup5Blue, shootPoseBlue))
                .setLinearHeadingInterpolation(pickup5Blue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain moveBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, moveBlue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), moveBlue.getHeading())
                .build();
    }


    // Convert RED pose to BLUE field pose
    public static Pose convertToBlue(Pose p) {
        return new Pose(p.getY(), -p.getX(), -p.getHeading());
    }
}
