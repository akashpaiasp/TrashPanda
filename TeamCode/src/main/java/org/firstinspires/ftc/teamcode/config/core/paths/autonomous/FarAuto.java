package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarAuto {

    // ---------------- RED POSES ----------------

    public static final Pose startPose = new Pose(14.25, -63, -.012);
    public static final Pose shootPose = new Pose(7.35, -55.7, 0.126);
    public static final Pose humanPlayer = new Pose(57.3, -63.3, -0.05);
    public static final Pose turn = new Pose(60.8, -60, -.189);

    private static final Pose strafe = new Pose(20, -33.5, 0);
    private static final Pose thirdSpike = new Pose(55, -36, 0);
    public static final Pose park = new Pose(53, -57.5, -.189);

    // ---------------- BLUE POSES ----------------

    // ---------------- BLUE POSES ----------------

    public static final Pose startPoseBlue = convertToBlue(startPose);
    public static final Pose shootPoseBlue = convertToBlue(shootPose);
    public static final Pose humanPlayerBlue = convertToBlue(humanPlayer);
    public static final Pose turnBlue = convertToBlue(turn);

    private static final Pose strafeBlue = convertToBlue(strafe);
    private static final Pose thirdSpikeBlue = convertToBlue(thirdSpike);
    public static final Pose parkBluePose = convertToBlue(park);

    // ---------------- RED PATHS ----------------

    public static PathChain shoot1(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
    }

    public static PathChain humanPlayerZone(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPose, humanPlayer))
                .setLinearHeadingInterpolation(shootPose.getHeading(), humanPlayer.getHeading())
                .build();
    }

    public static PathChain shootHumanPlayer(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(humanPlayer, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
    }

    public static PathChain spikeMarkPickup(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose, strafe, thirdSpike))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shootSpikeMark(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(thirdSpike, shootPose))
                .setLinearHeadingInterpolation(thirdSpike.getHeading(), shootPose.getHeading())
                .build();
    }

    public static PathChain ballCVIntake(Follower f, double ballY) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(humanPlayer.getX(), ballY, humanPlayer.getHeading()), new Pose(humanPlayer.getX(), ballY, humanPlayer.getHeading())))
                .setTangentHeadingInterpolation()
                .build();
    }
    public static PathChain ballCVShoot(Follower f, Pose currentPose) {
        return f.pathBuilder()
                .addPath(new BezierLine(currentPose, shootPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), shootPose.getHeading())
                .build();
    }
    public static PathChain park(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(humanPlayer, park))
                .setLinearHeadingInterpolation(humanPlayer.getHeading(), park.getHeading())
                .build();
    }

    // ---------------- BLUE PATHS ----------------

    // ---------------- BLUE PATHS ----------------

    public static PathChain shoot1Blue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, shootPoseBlue))
                .setConstantHeadingInterpolation(shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain humanPlayerZoneBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(shootPoseBlue, humanPlayerBlue))
                .setLinearHeadingInterpolation(shootPoseBlue.getHeading(), humanPlayerBlue.getHeading())
                .build();
    }

    public static PathChain shootHumanPlayerBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(humanPlayerBlue, shootPoseBlue))
                .setConstantHeadingInterpolation(shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain spikeMarkPickupBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPoseBlue, strafeBlue, thirdSpikeBlue))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain shootSpikeMarkBlue(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(thirdSpikeBlue, shootPoseBlue))
                .setLinearHeadingInterpolation(thirdSpikeBlue.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    public static PathChain ballCVIntakeBlue(Follower f, double ballY) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shootPoseBlue, new Pose(humanPlayerBlue.getX(), ballY, humanPlayerBlue.getHeading()), new Pose(humanPlayerBlue.getX(), ballY, humanPlayerBlue.getHeading())))
                .setTangentHeadingInterpolation()
                .build();
    }
    public static PathChain ballCVShootBlue(Follower f, Pose currentPose) {
        return f.pathBuilder()
                .addPath(new BezierLine(currentPose, shootPoseBlue))
                .setLinearHeadingInterpolation(currentPose.getHeading(), shootPoseBlue.getHeading())
                .build();
    }

    // ---------------- CONVERSION ----------------

    public static Pose convertToBlue(Pose p) {
        return new Pose(-p.getX(), p.getY(), Math.PI - p.getHeading());
    }

    public static Pose getCVPose(Pose og, Pose cv) {
        return new Pose(og.getX(), cv.getY(), og.getHeading());
    }
}