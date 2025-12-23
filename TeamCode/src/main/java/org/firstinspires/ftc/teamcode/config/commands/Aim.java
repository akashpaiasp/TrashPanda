package org.firstinspires.ftc.teamcode.config.commands;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

public class Aim extends CommandBase {
    private Robot r;
    private double targetX;
    private double targetY;
    public static double fudgeFactor = 0;
    public static double flightTime = 0.5;

    private static final double MIN_ANGLE = -90; // turret left limit
    private static final double MAX_ANGLE = 90;  // turret right limit
    public Aim(Robot r, double targetX, double targetY) {
        this.r = r;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    @Override
    public void execute() {
        /*
         * Calculates the turret angle relative to the robot's front (degrees).
         * Clamps to [-90°, +90°].
         */

        double vx = r.getFollower().getVelocity().getXComponent();
        double vy = r.getFollower().getVelocity().getYComponent();
        double va = 0;//r.getFollower().getAngularVelocity();

        double dx;
        double dy;
        if (Robot.alliance == Alliance.RED) {
            dx = targetX - r.getFollower().getPose().getX() - vx * flightTime;
            dy = targetY - r.getFollower().getPose().getY() - vy * flightTime;
        }
        else {
            dx = -(targetX - r.getFollower().getPose().getX() - vx * flightTime);
            dy = targetY + r.getFollower().getPose().getY() - vy * flightTime;
        }
        double robotHeading = Math.toDegrees(r.getAlliance() == Alliance.RED ? r.getFollower().getPose().getHeading() - flightTime * va: r.getFollower().getPose().getHeading() + Math.toRadians(180) + flightTime * va);

        double angleToTargetField = Math.toDegrees(Math.atan2(dy, dx));

        double turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading + fudgeFactor) ;

        turretRelativeAngle = Range.clip(turretRelativeAngle, MIN_ANGLE, MAX_ANGLE);
        //turretRelativeAngle = 0;

        r.turret.setTargetDegrees(-turretRelativeAngle);

        r.getTelemetry().addData("Target Degrees", -turretRelativeAngle);

    }

    @Override
    public void end(boolean interrupted) {
        /*
        if (interrupted) {
            r.turret.usePID = false;
            r.turret.power = 0;
        } */
    }

    public boolean isFinished() {
        return true;
    }

    private double wrapTo180(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }
}