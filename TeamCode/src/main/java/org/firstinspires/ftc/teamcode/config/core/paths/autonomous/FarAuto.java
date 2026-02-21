package org.firstinspires.ftc.teamcode.config.core.paths.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


public class FarAuto {
    // Red Poses
        public static final Pose startPose = new Pose(13, -60.6, 0);
    public static final Pose shootPose = new Pose(13, -57, 0);
    public static final Pose humanPlayer = new Pose(59, -57.5, -.31);
    public static final Pose turn = new Pose(60.8, -61.6, -.189);

    private static final Pose strafe = new Pose(20, -33.5, 0);
    private static final Pose thirdSpike = new Pose(53, -33.5, 0);





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
                /*.addPath(new BezierLine(humanPlayer, turn))
                .setLinearHeadingInterpolation(humanPlayer.getHeading(), turn.getHeading()) */
                .build();
    }

    public static PathChain shootHumanPlayer(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierLine(turn, shootPose))
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





    // Convert RED pose to BLUE field pose
    public static Pose convertToBlue(Pose p) {
        return new Pose(-p.getX(), p.getY(),  Math.PI - p.getHeading());
    }
}