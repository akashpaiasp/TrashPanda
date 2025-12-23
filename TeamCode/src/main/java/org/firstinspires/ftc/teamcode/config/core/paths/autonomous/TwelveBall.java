package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class TwelveBall {
    // Red Poses
    public static final Pose startPose = new Pose(51.2, -50.2, -0.83);

    private static final Pose shootPose = new Pose(15.5, -22, -1.57);
    private static final Pose pickup1 = new Pose(14.6, -48.5, -1.567);
    private static final Pose strafeGate = new Pose(6.57, -50, -1.554);
    private static final Pose gate = new Pose(-6, -53, -1.56);
    private static final Pose strafe1 = new Pose(-9.25, -25, -1.58);
    private static final Pose pickup2 = new Pose(-9.11, -50, -1.56);
    private static final Pose strafe2 = new Pose(-32.38, -25, -1.57);
    private static final Pose pickup3 = new Pose(-33.5, -53, -1.59);
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
    private static final Pose move = new Pose(5, -29.24, -1.588);

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


    // -------------------- RED PATHS --------------------

    public static PathChain shoot1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain pickup1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, pickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1.getHeading())
                .build();
    }

    public static PathChain gate(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(pickup1, strafeGate))
                .setLinearHeadingInterpolation(pickup1.getHeading(), strafeGate.getHeading())
                .addPath(new BezierCurve(strafeGate, gate))
                .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                .build();
    }

    public static PathChain shoot2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, shootPose))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain shoot215(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup1, shootPose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain strafe1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe1.getHeading())
                .build();
    }

    public static PathChain pickup2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(strafe1, pickup2))
                .setLinearHeadingInterpolation(strafe1.getHeading(), pickup2.getHeading())
                .build();
    }

    public static PathChain gate15(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(pickup2, gate))
                .setLinearHeadingInterpolation(pickup2.getHeading(), gate.getHeading())
                .build();
    }

    public static PathChain gate18(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose, strafeGate18))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafeGate18.getHeading())
                .addPath(new BezierCurve(strafeGate18, openGate18))
                .setLinearHeadingInterpolation(strafeGate18.getHeading(), openGate18.getHeading())
                .build();
    }

    public static PathChain intakeGate18(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(openGate18, moveToIntake18))
                .setLinearHeadingInterpolation(openGate18.getHeading(), moveToIntake18.getHeading())
                .addPath(new BezierCurve(moveToIntake18, intakeGate18))
                .setLinearHeadingInterpolation(moveToIntake18.getHeading(), intakeGate18.getHeading())
                .build();
    }

    public static PathChain shoot18(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(intakeGate18, shootPose))
                .setLinearHeadingInterpolation(intakeGate18.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain shoot3(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(pickup2, shootPose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain shoot315(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(gate, shootPose))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain strafe2(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe2.getHeading())
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
                .addPath(new BezierLine(pickup3, shootPose))
                .setLinearHeadingInterpolation(pickup3.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain pickup4(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe3.getHeading())
                .addPath(new BezierLine(strafe3, strafe35))
                .setLinearHeadingInterpolation(strafe3.getHeading(), strafe35.getHeading())
                .addPath(new BezierLine(strafe35, strafe36))
                .setLinearHeadingInterpolation(strafe35.getHeading(), strafe36.getHeading())
                .addPath(new BezierLine(strafe36, pickup4))
                .setLinearHeadingInterpolation(strafe36.getHeading(), pickup4.getHeading())
                .build();
    }

    public static PathChain shoot5(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup4, shootPose))
                .setLinearHeadingInterpolation(pickup4.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain pickup5(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, strafe4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), strafe4.getHeading())
                .addPath(new BezierLine(strafe4, pickup5))
                .setLinearHeadingInterpolation(strafe4.getHeading(), pickup5.getHeading())
                .build();
    }

    public static PathChain shoot6(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(pickup5, shootPose))
                .setLinearHeadingInterpolation(pickup5.getHeading(), shootPose.getHeading())
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
