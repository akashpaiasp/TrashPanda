package org.firstinspires.ftc.teamcode.config.util;

import static org.firstinspires.ftc.teamcode.config.subsystems.Hood.shootingVariable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Hood;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.opencv.core.Mat;
@Config
public class KinematicsCalculator {
    //50 - 1.76, .35
        //83 - 1.45, .65
    //92 - 1.45, .
    //120 - 1.45, .65
    //143 - 1.25, .72


    public static final double y_exit_in = 13.528;          // shooter exit height
    public static  double targetAuto = 35;
    public static  double targetTele = 33;
    public static double y_target_in = 36;          // goal height
    public static double y_target_in_airsort = 28;
    public static final double d_flywheel_in = 2.835;       // flywheel diameter
    public static final double r_flywheel_in = d_flywheel_in / 2.0;

    public static final double g = 9.81;                     // gravity (m/s^2)


    public static  double efficiency = .83;          // launcher efficiency factor
    public static double efficiencyTele = .83;
    public static double efficiencyAuto = .83;

    public static boolean manualFudge = false;

    public static double FUDGE_FACTOR_VEL = 1.45;
    public static double AUTO_FUDGE = 1.45;

    // Converted constants (meters)
    public static final double y_exit_m = inchesToMeters(y_exit_in);
    public static double y_target_m = inchesToMeters(y_target_in);
    public static final double r_flywheel_m = inchesToMeters(r_flywheel_in);
    public static final double max_angle = 58;
    public static double min_angle = 40;

    public static double max_rpm = 4200.0;
    public static double min_rpm = 500.0;
    public static double entrance_angle = -30;
    public static boolean manual_angle = false;

    private double distance;
    private double RPM = 0;
    private double angleDeg = 45;
    public static boolean airsort = false;
    public boolean lob = false;

    public static double hoodVariable = .37;
    public static boolean manualHood = false;
    public static double mediumHood = .65;
    public static double closeHood = .35;
    public static double farHood = .72;



    public KinematicsCalculator(double distanceToGoal){
        distance = distanceToGoal;
    }


    public void setDistance(double distance) {
        this.distance = inchesToMeters(distance);
        if (airsort) {
            y_target_m = inchesToMeters(y_target_in_airsort);
        }
        else {
            if (distance > inchesToMeters(35))
                y_target_m = inchesToMeters(y_target_in);
            else
                y_target_m = inchesToMeters(y_target_in);
        }
    }

    public double getRPM() {
        if (!manualFudge) {
            if(distance < inchesToMeters(70)) {
                FUDGE_FACTOR_VEL = 1.76;
                if (!Launcher.manualCounterRoller) {
                    Launcher.counterRollerPower = .91;
                }
            }
            else if (distance > inchesToMeters(120)) {
                if (!Launcher.manualCounterRoller) {
                    Launcher.counterRollerPower = 1;
                }
                FUDGE_FACTOR_VEL = 1.25;
            }
            else {
                if (!Launcher.manualCounterRoller) {
                    Launcher.counterRollerPower = .91;
                }
                FUDGE_FACTOR_VEL = 1.45;
            }
        }
        if (airsort) {
            min_angle = 38;
            double thetaRad = Math.toRadians((max_angle + min_angle) / 2);
            double vel = Math.sqrt(g * distance * distance /
                    (2.0 * Math.pow(Math.cos(thetaRad), 2.0) *
                            (distance * Math.tan(thetaRad) + y_exit_m - y_target_m))
            );
            RPM = velToRpm(vel);
            return velToRpm(vel);
        }
        double vel;
        double thetaRad1, thetaRad2;
        if (distance <= inchesToMeters(30)) {
            return 900;
            /*thetaRad1 = Math.toRadians(min_angle);
            thetaRad2 = Math.toRadians(min_angle + 1); */
        }
        else {
            thetaRad1 = Math.toRadians(hoodToTheta(hoodVariable));//(max_angle + min_angle) / 2.0);
            //thetaRad1 = Math.toRadians(max_angle);
            //thetaRad2 = Math.toRadians(min_angle);
        }

        double vel1 = Math.sqrt(g * distance * distance /
                (2.0 * Math.pow(Math.cos(thetaRad1), 2.0) *
                        (distance * Math.tan(thetaRad1) + y_exit_m - y_target_m))
        );

        /*double vel2 = Math.sqrt(g * distance * distance /
                (2.0 * Math.pow(Math.cos(thetaRad2), 2.0) *
                        (distance * Math.tan(thetaRad2) + y_exit_m - y_target_m))
        ); */
        /*if (distance >= inchesToMeters(35))
            vel = Math.max(vel1, vel2);
        else {
            vel = //Math.min(vel1, vel2);
            rpmToVel(2600);
        }*/

        /*
        if(isValid(vel1) && isValid(vel2) || (!isValid(vel1) && !isValid(vel2))) {
            vel = (vel1 + vel2) / 2;
        }
        else if (isValid(vel1)) vel = vel1;
        else vel = vel2; */

        vel = vel1;
        RPM = velToRpm(vel);
        return velToRpm(vel);
    }

    public double getHood(double currentRPM) {
        if (!manualHood) {
            if (distance > inchesToMeters(120))
                hoodVariable = farHood;
            else if (distance < inchesToMeters(70))
                hoodVariable = closeHood;
            else hoodVariable = mediumHood;
        }
        //currentRPM = Math.round(currentRPM * )
        if (airsort) {
            return lob ? Hood.hoodDown : Hood.hoodUp;
        }
        //RPM = currentRPM;
        if (distance < inchesToMeters(45)) return Hood.hoodDown;
        double v0 = rpmToVel(currentRPM);
        //double v0 = rpmToVel(RPM);

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
        /*if (distance > inchesToMeters(35))
            return Hood.hoodUp; */
        //return thetaToHood(thetaDeg);
        return hoodVariable;
    }




    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private static double rpmToVel(double rpm) {

        return(Math.PI * r_flywheel_m * rpm / 60.0) * efficiency * FUDGE_FACTOR_VEL;
        //return 0.0008 * rpm * FUDGE_FACTOR_VEL;
    }

    private static double velToRpm(double vel) {
        if (Launcher.teleop)
            return vel / (Math.PI * r_flywheel_m / 60.0 * efficiency * FUDGE_FACTOR_VEL);
        return vel / (Math.PI * r_flywheel_m / 60.0 * efficiency * AUTO_FUDGE);
        //return vel / .0008 / FUDGE_FACTOR_VEL;
    }

    private static double thetaToHood(double theta) {
        if (theta < 0) return -1;
        return Range.clip((72.77 - theta) / 42.02, Hood.hoodDown, Hood.hoodUp);
    }

    public static double hoodToTheta(double hood) {
        return Range.clip(72.77 - 42.02 * (hood), min_angle, max_angle);
    }

    public  double getFlightTime() {
        //return 0 ;


        double theta = Math.toRadians(angleDeg);
        double v0 = rpmToVel(RPM);

        double v_horizontal = v0 * Math.cos(theta);
        double ft = distance / v_horizontal;
        if (Double.isNaN(ft) || ft > 10 || ft < 0) return 0;
        return ft;
    }

    public boolean isValid(double vel) {
        return vel > rpmToVel(min_rpm) && vel < rpmToVel(max_rpm);
    }
}
