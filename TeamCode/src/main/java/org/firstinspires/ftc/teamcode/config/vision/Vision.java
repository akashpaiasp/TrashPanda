package org.firstinspires.ftc.teamcode.config.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Vision {

    // --- CAMERA MOUNT OFFSET CONFIGURATION ---
    // Measure these physically on your robot (in millimeters or inches, just keep it consistent with REAL_BALL_WIDTH_MM)
    // Assuming Y is forward, X is right relative to the center of the robot.
    public static final double CAMERA_OFFSET_X = 150.0; // Example: Camera is 150mm to the right of center
    public static final double CAMERA_OFFSET_Y = 200.0; // Example: Camera is 200mm forward of center
    public static final double CAMERA_ANGLE_OFFSET = 0.0; // Example: Camera faces perfectly forward (0 degrees)

    private ArrayList<VisionBall> ballsInMemory = new ArrayList<>();

    public void processCurrentBalls(List<DetectorResult> rawData, double robotX, double robotY, double robotHeading) {

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
                if (ball.getTicksSinceLastSeen() > 10000) {
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
}