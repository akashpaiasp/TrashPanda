package org.firstinspires.ftc.teamcode.config.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import java.util.List;

public class VisionBall {

    // Constants for Distance Math
    public static final double REAL_BALL_WIDTH_MM = 89.0; // Approx FTC Game Element size
    public static final double FOCAL_LENGTH_PIXELS = 800.0; //FocalLength = (realDistance * pixelW) / RealWidth

    public enum Color { GREEN, PURPLE, UNKNOWN }

    private Color color;
    private double tx, ty; // Target angles in degrees
    private double pixelX, pixelY, pixelW, pixelH;

    // Positions
    private double distance;
    private double relativeX, relativeY; // Position relative to the center of the ROBOT
    private double absoluteX, absoluteY; // Position on the FTC field

    // Lifecycle Tracking
    private int consecutiveTicksSeen = 1;
    private int ticksSinceLastSeen = 0;
    private boolean isConfirmed = false;

    //constructor based on limelight data
    public VisionBall(DetectorResult rawBall, double robotX, double robotY, double robotHeading,
                      double camOffsetX, double camOffsetY, double camAngleOffset) {
        classifyBallColor(rawBall.getClassName());
        updateWithNewData(rawBall, robotX, robotY, robotHeading, camOffsetX, camOffsetY, camAngleOffset);
    }

    public void updateWithNewData(DetectorResult rawBall, double robotX, double robotY, double robotHeading,
                                  double camOffsetX, double camOffsetY, double camAngleOffset) {
        this.ticksSinceLastSeen = 0;
        this.tx = rawBall.getTargetXDegrees();
        this.ty = rawBall.getTargetYDegrees();

        // Calculate Bounding Box
        List<List<Double>> corners = rawBall.getTargetCorners();
        if (corners != null && corners.size() == 4) {
            double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
            double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;

            for (List<Double> corner : corners) {
                double x = corner.get(0);
                double y = corner.get(1);
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
            this.pixelW = maxX - minX;
            this.pixelH = maxY - minY;
            this.pixelX = minX + (this.pixelW / 2.0);
            this.pixelY = minY + (this.pixelH / 2.0);
        }

        updatePositions(robotX, robotY, robotHeading, camOffsetX, camOffsetY, camAngleOffset);
    }

    private void updatePositions(double robotX, double robotY, double robotHeading,
                                 double camOffsetX, double camOffsetY, double camAngleOffset) {

        // 1. Distance from the Camera Lens
        this.distance = (REAL_BALL_WIDTH_MM * FOCAL_LENGTH_PIXELS) / Math.max(pixelW, 1.0);

        // 2. Position relative to the CAMERA (Assuming Y is forward, X is right)
        double txRad = Math.toRadians(this.tx);
        double camRelX = this.distance * Math.sin(txRad);
        double camRelY = this.distance * Math.cos(txRad);

        // 3. Position relative to ROBOT CENTER
        // First, rotate the coordinates if the camera is mounted at an angle (yaw)
        double camAngleRad = Math.toRadians(camAngleOffset);
        double rotatedX = camRelX * Math.cos(camAngleRad) + camRelY * Math.sin(camAngleRad);
        double rotatedY = -camRelX * Math.sin(camAngleRad) + camRelY * Math.cos(camAngleRad);

        // Next, add the physical X/Y offset of the camera from the robot's center
        this.relativeX = rotatedX + camOffsetX;
        this.relativeY = rotatedY + camOffsetY;

        // 4. Absolute Position on Field
        double headingRad = Math.toRadians(robotHeading);
        this.absoluteX = robotX + (this.relativeX * Math.cos(headingRad) - this.relativeY * Math.sin(headingRad));
        this.absoluteY = robotY + (this.relativeX * Math.sin(headingRad) + this.relativeY * Math.cos(headingRad));
    }

    public boolean isSameBall(DetectorResult rawBall) {
        // 1. Spatial Check: Is it physically close to where we last saw it?
        double angleThreshold = 4.0;
        double diffX = Math.abs(this.tx - rawBall.getTargetXDegrees());
        double diffY = Math.abs(this.ty - rawBall.getTargetYDegrees());
        boolean isClose = (diffX < angleThreshold && diffY < angleThreshold);

        // 2. Color Check: Does the detected label match our stored color?
        String rawName = rawBall.getClassName();
        boolean colorMatches = false;

        if (this.color == Color.PURPLE && rawName.equalsIgnoreCase("purple")) {
            colorMatches = true;
        } else if (this.color == Color.GREEN && rawName.equalsIgnoreCase("green")) {
            colorMatches = true;
        }

        // Only return true if it's both in the same spot AND the same color
        return isClose && colorMatches;
    }

    private void classifyBallColor(String name) {
        if (name == null) {
            this.color = Color.UNKNOWN;
            return;
        }

        switch (name.toLowerCase()) {
            case "purple":
                this.color = Color.PURPLE;
                break;
            case "green":
                this.color = Color.GREEN;
                break;
            default:
                this.color = Color.UNKNOWN;
                break;
        }
    }

    // --- Lifecycle Methods ---
    public void incrementTicksSinceLastSeen() { this.ticksSinceLastSeen++; }
    public void incrementConsecutiveTicks() {
        this.consecutiveTicksSeen++;
        if (this.consecutiveTicksSeen >= 5) this.isConfirmed = true;
    }
    public void resetConsecutiveTicks() { this.consecutiveTicksSeen = 0; }

    public int getTicksSinceLastSeen() { return ticksSinceLastSeen; }
    public boolean isConfirmed() { return isConfirmed; }

    // --- Getters ---
    public Color getColor() { return color; }
    public double getDistance() { return distance; }
    public double getRelativeX() { return relativeX; }
    public double getRelativeY() { return relativeY; }
    public double getAbsoluteX() { return absoluteX; }
    public double getAbsoluteY() { return absoluteY; }
    public double[] getPixelBoundingBox() { return new double[]{pixelX, pixelY, pixelW, pixelH}; }
}
