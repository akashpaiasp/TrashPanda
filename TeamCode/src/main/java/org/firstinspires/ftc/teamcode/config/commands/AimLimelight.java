package org.firstinspires.ftc.teamcode.config.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;

@Config
public class AimLimelight extends CommandBase {
    private Robot r;

    private static final double MIN_ANGLE = -90; // turret left limit
    private static final double MAX_ANGLE = 90;  // turret right limit
    public static double p = 0.03, i = 0, d = 1, f = 0, l = 0.005;
    public double turretRelativeAngle = 0;

    public AimLimelight(Robot r) {
        this.r = r;
    }

    @Override
    public void execute() {
        /*
         * Calculates the turret angle relative to the robot's front (degrees).
         * Clamps to [-90°, +90°].
         */
        if (r.limelight.getResult().isValid()) {
            double angle = r.limelight.getResult().getTx();
            //turretRelativeAngle = 0;

            if (!((r.turret.getTotalDegrees() > 90 && angle < 0) || r.turret.getTotalDegrees() < -90 && angle > 0))
                r.turret.updateLL(angle);
        }

        r.getTelemetry().addData("Target Degrees", turretRelativeAngle);

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