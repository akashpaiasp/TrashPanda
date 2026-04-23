package org.firstinspires.ftc.teamcode.config.vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

@Config
public class Vision {

    // --- CAMERA MOUNT OFFSET CONFIGURATION ---
    // Measure these physically on your robot (in millimeters or inches, just keep it consistent with REAL_BALL_WIDTH_MM)
    // Assuming Y is forward, X is right relative to the center of the robot.
    public static final double CAMERA_OFFSET_X = 0; // Example: Camera is 150mm to the right of center
    public static final double CAMERA_OFFSET_Y = 200.0; // Example: Camera is 200mm forward of center
    public static  double CAMERA_ANGLE_OFFSET = 0.0; // Example: Camera faces perfectly forward (0 degrees)
    public static  double INTAKE_REACH = 8.0; // inches, measure from robot center to intake contact point
    public static VisionBall ball1, ball2, ball3;


    private ArrayList<VisionBall> ballsInMemory = new ArrayList<>();

    public void processCurrentBalls(List<DetectorResult> rawData, double robotX, double robotY, double robotHeading) {
        robotX = inToMm(robotX);
        robotY = inToMm(robotY);
        ArrayList<VisionBall> successfullyUpdatedBalls = new ArrayList<>();

        if (rawData != null) {
            for (DetectorResult rawBall : rawData) {
                boolean matchedToExisting = false;

                for (VisionBall memoryBall : ballsInMemory) {
                    if (memoryBall.isSameBall(rawBall)) {
                        memoryBall.updateWithNewData(rawBall, robotX, robotY, robotHeading,
                                CAMERA_OFFSET_X, CAMERA_OFFSET_Y, CAMERA_ANGLE_OFFSET);
                        memoryBall.incrementConsecutiveTicks();
                        successfullyUpdatedBalls.add(memoryBall);
                        matchedToExisting = true;
                        break;
                    }
                }

                if (!matchedToExisting) {
                    VisionBall newBall = new VisionBall(rawBall, robotX, robotY, robotHeading,
                            CAMERA_OFFSET_X, CAMERA_OFFSET_Y, CAMERA_ANGLE_OFFSET);
                    ballsInMemory.add(newBall);
                    successfullyUpdatedBalls.add(newBall);
                }
            }
        }

        Iterator<VisionBall> iterator = ballsInMemory.iterator();
        while (iterator.hasNext()) {
            VisionBall ball = iterator.next();

            if (!successfullyUpdatedBalls.contains(ball)) {
                ball.incrementTicksSinceLastSeen();
                ball.resetConsecutiveTicks();
            }

            if (ball.isConfirmed()) {
                if (ball.getTicksSinceLastSeen() > 100) {
                    iterator.remove();
                }
            } else {
                if (ball.getTicksSinceLastSeen() > 100) {
                    iterator.remove();
                }
            }
        }
    }

    public ArrayList<VisionBall> getCachedBalls() {
        return ballsInMemory;
    }

    public void clearCache() {
        ballsInMemory.clear();
    }

    public double inToMm(double in) {
        return in * 25.4;
    }

    public static double mmToIn(double mm) {
        return mm / 25.4;
    }

    private Pose getIntakeTarget(Pose from, VisionBall ball) {
        double bx = ball.getAbsoluteX();
        double by = ball.getAbsoluteY();

        double dx = bx - from.getX();
        double dy = by - from.getY();

        double dist = Math.hypot(dx, dy);

        // if we're basically already on it, just return ball position
        if (dist < 1e-6) {
            return new Pose(bx, by);
        }

        double ux = dx / dist;
        double uy = dy / dist;

        // back off from the ball by intake reach
        double tx = bx - ux * INTAKE_REACH;
        double ty = by - uy * INTAKE_REACH;

        return new Pose(tx, ty);
    }

    public void getBalls() {
        ball1 = ballsInMemory.get(0);
        ball2 = ballsInMemory.get(1);
        ball3 = ballsInMemory.get(2);
    }

    public PathChain getBallPath(Follower f) {
        getBalls();
        ballsInMemory.sort(Comparator.comparingDouble(VisionBall::getDistance));

        Pose start = f.getPose();



        Pose first = getIntakeTarget(start,ball1);
        Pose second = getIntakeTarget(first, ball2);
        Pose third = getIntakeTarget(second, ball3);
        return f.pathBuilder()
                /*.addPath(new BezierCurve(start, ball1, ball2, ball3))
                .setTangentHeadingInterpolation() */
                .addPath(new BezierLine(start, first))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(first, second))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(second, third))
                .setTangentHeadingInterpolation()
                .build();
    }

}