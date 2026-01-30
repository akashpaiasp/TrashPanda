package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.flightTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.KinematicsCalculator;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;
import org.firstinspires.ftc.teamcode.config.util.AxonContinuous;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;
import org.opencv.core.Mat;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config
public class Turret extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    public static double power = 0;
    public boolean turretOffAuto = false;
    public boolean lockTurret = false;

    public static double offset = -4;
    //61.7, 14.9
    //public static boolean powerMode = false;

    // public static double turretPosConstant = 0.51;
    // public boolean first = false;

    public static double p = 0.01, i = 0, d = 0.4, f = 0, l = 0.045;
    public static double p2 = 0.005, i2 = 0, d2 = 1, f2 = 0, l2 = 0.005;
    public static double deadZone = 0.6;

    public PDFLController controller;
    public PDFLController llcontroller;

    public static double target = 0.0;
    public static double GEAR_RATIO = 66.0/115.0;
    private double targetX;
    private double targetY;
    private Pose botPose;
    public static double fudgeFactor = 0;
    public static boolean useTurret = true;


    public static  double MIN_ANGLE = -120; // turret left limit
    public static  double MAX_ANGLE = 120;  // turret right limit
    public static double autoFudge = 3;
    public double current;

    private MultipleTelemetry telemetry;
    public AxonContinuous spin;
    public CRServo spin2;

    public static double targetRange = 3;
    //public Servo spin;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spin = new AxonContinuous(hardwareMap, "sh1", "ca1");
        spin.getC().setDirection(DcMotorSimple.Direction.REVERSE);
        spin2 = hardwareMap.get(CRServo.class, "sh0");
        spin2.setDirection(DcMotorSimple.Direction.REVERSE);
        //spin = hardwareMap.get(Servo.class, "sh2");
        controller = new PDFLController(p, d, f, l, i);
        controller.setDeadZone(deadZone);
        llcontroller = new PDFLController(p2, d2, f2, l2, i2);
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void periodicTest() {
        spin.calculate();
        current = -getTotalDegrees();
        controller.updateConstants(p, d, target > current ? f : -f, l, i);
        controller.update(current, target);
        power = controller.run();

        power = Range.clip(power, -1, 1);

        spin.setPower(power);
        spin2.setPower(power);
        controller.setDeadZone(deadZone);

        telemetry.addData("Rise Time", controller.getRiseTime());
        telemetry.addData("Settling Time", controller.getSettlingTime());
        telemetry.addData("Settled", controller.isSettled());
        telemetry.addData("Target", target);
        telemetry.addData("Current", current);
        telemetry.addData("Power", power);
        telemetry.addData("Raw", spin.getVolts());
        telemetry.addData("Rotations", totalRotations());
        telemetry.update();
    }

    public void periodicTest2() {
        spin.calculate();

        spin.setPower(power);
        spin2.setPower(power);

        telemetry.addData("Raw", spin.getVolts());
        telemetry.addData("Rotations", spin.getNumRotations());
        telemetry.addData("Partial rotations", spin.getPartial_rotations());
        telemetry.addData("Full rotations", spin.getFull_rotations());
        telemetry.update();
    }



    public void periodic() {
        //if (Robot.logData) log();
        aim();
        spin.calculate();
        current = -getTotalDegrees();
        controller.update(current, target);
        llcontroller.updateConstants(p2, d2, f2, l2, i2);

        power = controller.run();

        power = Range.clip(power, -1, 1);

        if (!lockTurret || true) {
            spin.setPower(power);
            spin2.setPower(power);
        }
        else {
            spin.setPower(0);
            spin2.setPower(0);
        }

        telemetry.addData("turret target", target);
        telemetry.addData("turret power", power);
        telemetry.addData("turret volts", spin.getVolts());
        //telemetry.addData("Use Limelight", limelightMode);

    }

    /*
    public void periodicTest() {
        spin.setPosition(turretPosConstant);
    } */

    /*
    public void init() {
        spin.setPosition(turretPosConstant);
    } */

    //Call this method to open/close the servos


    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */

    /*
    public void periodicTest() {
        if (powerMode)
            spin.setPower(power);
        spin.calculate();
        telemetry.addData("Servo Raw", spin.getVolts());
        telemetry.addData("Degrees", getDegrees());
        telemetry.addData("Total Degrees", getTotalDegrees());
        telemetry.update();

    } */
    //0 - 3.3 = full revolutions

    public double totalRotations() {
        return servoToBelt(spin.getNumRotations());
    }

    public double servoToBelt(double servo) {
        return servo * GEAR_RATIO;
    }

    public double getDegrees() {
        return ((totalRotations() * 360) - offset) % 360;
    }

    public double getTotalDegrees() {
        return -(totalRotations() * 360 - offset);
    }

    public double getRadians() {
        return (getDegrees() * Math.PI  / 180.0) % (Math.PI * 2.0);
    }

    public void setTargetDegrees(double targetDeg) {
        target = targetDeg;
        turretOffAuto = true;
    }

    public void updateAiming(double targetX, double targetY, Pose botPose) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.botPose = botPose;
    }



    public void aim() {
        /*
         * Calculates the turret angle  relative to the robot's front (degrees).
         * Clamps to [-90°, +90°].
         */

        double vxTemp = 0;//KinematicsCalculator.inchesToMeters(r.getFollower().getVelocity().getXComponent());
        double vx = Double.isNaN(vxTemp) ? 0 : vxTemp;
        double vyTemp = 0;//KinematicsCalculator.inchesToMeters(r.getFollower().getVelocity().getYComponent());
        double vy = Double.isNaN(vyTemp) ? 0 : vyTemp;
        double va = 0;//r.getFollower().getAngularVelocity();

        double dx;
        double dy;

        double x = botPose.getX();
        double y = botPose.getY();
        dx = targetX - x - vx * flightTime;
        dy = targetY - y - vy * flightTime;
        double robotHeading = Math.toDegrees(botPose.getHeading());

        double angleToTargetField = Math.toDegrees(Math.atan2(dy, dx));
        double turretRelativeAngle;

        if (Launcher.teleop || true)
            turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading + fudgeFactor) ;
        else {
            if (Robot.alliance == Alliance.RED)
                turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading + autoFudge *1.8);
            else
                turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading - autoFudge * 1.8);
        }

        turretRelativeAngle = Range.clip(turretRelativeAngle, MIN_ANGLE, MAX_ANGLE);
        //turretRelativeAngle = 0;
        if (useTurret) {
            if (Launcher.teleop || !turretOffAuto)
                target = -turretRelativeAngle;
        }
        else
            target = 0;

        telemetry.addData("Target Degrees", -turretRelativeAngle);

    }

    public boolean atTarget() {
        return Math.abs(target - current) < targetRange;
    }





    private double wrapTo180(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }
    public void updateLL(double d) {
        llcontroller.update(d, 1);
    }

    public void log() {
        Logger.logData(LogType.TURRET_TARGET, String.valueOf(target));
        Logger.logData(LogType.TURRET_VOLTS, String.valueOf(spin.getVolts()));
        Logger.logData(LogType.TURRET_PREV, String.valueOf(spin.lastVoltage));
        Logger.logData(LogType.TURRET_FULL_ROTS, String.valueOf(spin.full_rotations));

    }


}