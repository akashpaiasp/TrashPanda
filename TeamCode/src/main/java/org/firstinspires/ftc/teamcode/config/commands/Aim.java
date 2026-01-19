package org.firstinspires.ftc.teamcode.config.commands;

import static org.firstinspires.ftc.teamcode.config.core.Robot.auto;
import static org.firstinspires.ftc.teamcode.config.core.Robot.flightTime;
import static org.firstinspires.ftc.teamcode.config.core.Robot.processNoiseHeading;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.KinematicsCalculator;

public class Aim extends CommandBase {
    private Robot r;
    private double targetX;
    private double targetY;
    public static double fudgeFactor = 0;


    private static final double MIN_ANGLE = -90; // turret left limit
    private static final double MAX_ANGLE = 90;  // turret right limit
    public static double autoFudge = 3;
    public Aim(Robot r, double targetX, double targetY) {
        this.r = r;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    @Override
    public void execute() {
        /*
         * Calculates the turret angle  relative to the robot's front (degrees).
         * Clamps to [-90°, +90°].
         */

        double vxTemp = KinematicsCalculator.inchesToMeters(r.getFollower().getVelocity().getXComponent());
        double vx = Double.isNaN(vxTemp) ? vxTemp : 0;
        double vyTemp = KinematicsCalculator.inchesToMeters(r.getFollower().getVelocity().getYComponent());
        double vy = Double.isNaN(vyTemp) ? vyTemp : 0;
        double va = 0;//r.getFollower().getAngularVelocity();

        double dx;
        double dy;

        double x = r.turretX;
        double y = r.turretY;
        dx = targetX - x - vx * flightTime;
        dy = targetY - y - vy * flightTime;
        double robotHeading = Math.toDegrees(r.getFollower().getHeading());

        double angleToTargetField = Math.toDegrees(Math.atan2(dy, dx));
        double turretRelativeAngle;

        if (r.launcher.teleop)
             turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading + fudgeFactor) ;
        else {
            if (Robot.alliance == Alliance.RED)
                turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading + autoFudge *1.8);
            else
                turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading - autoFudge * 1.8);
        }

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