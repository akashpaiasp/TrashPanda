package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Hood;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.opencv.core.Mat;
@Config
public class KinematicsCalculator {
    public static final double y_exit_in = 13.528;          // shooter exit height
    public static  double targetAuto = 35;
    public static  double targetTele = 33;
    public static  double y_target_in = 33;          // goal height
    public static final double d_flywheel_in = 2.835;       // flywheel diameter
    public static final double r_flywheel_in = d_flywheel_in / 2.0;

    public static final double g = 9.81;                     // gravity (m/s^2)
    public static final double efficiency = .8;          // launcher efficiency factor

    public static double FUDGE_FACTOR_VEL = 0.946;

    // Converted constants (meters)
    public static final double y_exit_m = inchesToMeters(y_exit_in);
    public static  double y_target_m = inchesToMeters(y_target_in);
    public static final double r_flywheel_m = inchesToMeters(r_flywheel_in);
    public static final double max_angle = 61;
    public static final double min_angle = 40;

    public static double max_rpm = 5100.0;
    public static double min_rpm = 2500.0;
    public static double entrance_angle = -30;
    public static boolean manual_angle = false;

    private double distance;
    private double RPM = 0;
    private double angleDeg = 45;


    public KinematicsCalculator(double distanceToGoal){
        distance = distanceToGoal;
    }


    public void setDistance(double distance) {
        this.distance = inchesToMeters(distance);
        if (distance > inchesToMeters(35))
            y_target_m = inchesToMeters(y_target_in);
        else
            y_target_m = inchesToMeters(y_target_in + 10);
    }

    public double getRPM() {
        double vel;
            double thetaRad1, thetaRad2;
            thetaRad1 = Math.toRadians(max_angle - 1);
            thetaRad2 = Math.toRadians(min_angle + 1);

            double vel1 = Math.sqrt(g * distance * distance /
                    (2.0 * Math.pow(Math.cos(thetaRad1), 2.0) *
                            (distance * Math.tan(thetaRad1) + y_exit_m - y_target_m))
            );

            double vel2 = Math.sqrt(g * distance * distance /
                    (2.0 * Math.pow(Math.cos(thetaRad2), 2.0) *
                            (distance * Math.tan(thetaRad2) + y_exit_m - y_target_m))
            );
            if (distance > inchesToMeters(35))
                vel = Math.max(vel1, vel2);
            else
                vel = Math.min(vel1, vel2);//rpmToVel(3000);
            return velToRpm(vel);
        }

    public double getHood(double currentRPM) {
        RPM = currentRPM;
        if (distance < inchesToMeters(25)) return Hood.hoodDown;
        double v0 = rpmToVel(currentRPM);

        // Quadratic coefficients in tan(theta)
        double A = (g * distance * distance) / (2.0 * v0 * v0);
        double B = -distance;
        double C = A + (y_target_m - y_exit_m);

        double discriminant = B * B - 4.0 * A * C;

        // No physical solution
        if (discriminant < 0) {
            return -1;
        }

        double sqrtDisc = Math.sqrt(discriminant);

        double tan1 = (-B + sqrtDisc) / (2.0 * A);
        double tan2 = (-B - sqrtDisc) / (2.0 * A);

        // Low-angle solution (usually optimal for consistency)
        double tanLow = Math.min(tan1, tan2);
        double thetaDeg = Math.toDegrees(Math.atan(tanLow));

        // Limits
        if (thetaDeg < min_angle || thetaDeg > max_angle) {
            return -1;
        }
        angleDeg = thetaDeg;

        return thetaToHood(thetaDeg);
    }




    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private static double rpmToVel(double rpm) {
        return(Math.PI * r_flywheel_m * rpm / 60.0) * efficiency * FUDGE_FACTOR_VEL;
    }

    private static double velToRpm(double vel) {
        return vel / (Math.PI * r_flywheel_m / 60.0 * efficiency * FUDGE_FACTOR_VEL);
    }

    private static double thetaToHood(double theta) {
        if (theta < 0) return -1;
        return Range.clip(1.4758 - 0.015625 * theta, Hood.hoodDown, Hood.hoodUp);
    }

    public static double hoodToTheta(double hood) {
        return Range.clip(94.452 - 64 * hood, min_angle, max_angle);
    }

    public  double getFlightTime() {
        //return .5;

        double theta = Math.toRadians(angleDeg);
        double v0 = rpmToVel(RPM);

        double v_horizontal = v0 * Math.cos(theta);
        double ft = distance / v_horizontal;
        if (Double.isNaN(ft)) return 0;
        return ft;
    }
}
