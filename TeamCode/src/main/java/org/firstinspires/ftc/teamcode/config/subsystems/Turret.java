package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.flightTime;
import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

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
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;
import org.firstinspires.ftc.teamcode.config.util.AxonContinuous;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config
public class Turret extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    public static double power = 0;
    public static boolean continuousMode = false;
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
    public static boolean testLash = false;
    public static double t1 = 0, t2 = 0, t3 = 0, t4 = 0;

    public PDFLController controller;
    public PDFLController llcontroller;

    public static double target = 0.0;
    public static double GEAR_RATIO = 66.0/115.0;
    private double targetX;
    private double targetY;
    private Pose botPose;
    public static double fudgeFactor = 0;
    public static boolean useTurret = true;

    public static double zeroPos = .485;
    public static double oneEightyPos = 0.86;
    /*
    public static double leftPos = .5;
    public static double rightPos = .5; */
    public static double pos = .5;
    public static double lashFix = 0.025;
    public static double lashOffset = 0;



    public static  double MIN_ANGLE = -181; // turret left limit
    public static  double MAX_ANGLE = 181;  // turret right limit
    public static double autoFudge = 3;
    public double current;

    private MultipleTelemetry telemetry;
    public AxonContinuous spin; //sh0
    public CRServo spin2; //sh1
    public Servo left, right, middle, other;

    public static double targetRange = 3;
    //public Servo spin;

    public static double weight = .3;
    public static double velWeight = .3;

    public static boolean sotm = true;
    public double vx, vy = 0;
    public double prevX = 0, prevY = 0;
    public double dvx, dvy = 0;
    public boolean poseMode = false;

    public static boolean power1 = true, power2 = true, power3 = true, power4 = true;
    public static boolean rev1 = false, rev2 = false, rev3 = false, rev4 = false;


    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (continuousMode) {
            spin = new AxonContinuous(hardwareMap, "sh1", "ca1");
            spin.getC().setDirection(DcMotorSimple.Direction.REVERSE);
            spin2 = hardwareMap.get(CRServo.class, "sh0");
            spin2.setDirection(DcMotorSimple.Direction.REVERSE);
            //this is the reverse of the other thing

        }
        else {
            left = hardwareMap.get(Servo.class, "es4");
            right = hardwareMap.get(Servo.class, "es5");
            middle = hardwareMap.get(Servo.class, "cs1");
            other = hardwareMap.get(Servo.class, "cs0");

            if (rev1) left.setDirection(Servo.Direction.REVERSE);
            if (rev2) right.setDirection(Servo.Direction.REVERSE);
            if (rev3) middle.setDirection(Servo.Direction.REVERSE);
            if (rev4) other.setDirection(Servo.Direction.REVERSE);


        }
        //spin = hardwareMap.get(Servo.class, "sh2");
        controller = new PDFLController(p, d, f, l, i);
        controller.setDeadZone(deadZone);
        llcontroller = new PDFLController(p2, d2, f2, l2, i2);
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void periodicTest() {
        if (continuousMode) {
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
        }

        else {
            double b = getPos();
            if (!testLash) {
                left.setPosition(b + lashFix);
                right.setPosition(b - lashFix);
                middle.setPosition(b);
                other.setPosition(b);
            }
            else {
                left.setPosition(t1);
                right.setPosition(t2);
                middle.setPosition(t3);
                other.setPosition(t4);
            }
            telemetry.addData("left", left.getPosition());
            telemetry.addData("right", right.getPosition());
            telemetry.addData("middle", middle.getPosition());
        }


        telemetry.update();
    }

    public void periodicTest2() {
        if (continuousMode) {
            spin.calculate();

            spin.setPower(power);
            spin2.setPower(power);

            telemetry.addData("Raw", spin.getVolts());
            telemetry.addData("Rotations", spin.getNumRotations());
            telemetry.addData("Partial rotations", spin.getPartial_rotations());
            telemetry.addData("Full rotations", spin.getFull_rotations());
            telemetry.update();
        }
        else {
            if (!testLash) {
                if (power1)
                    left.setPosition(pos + lashFix);
                if (power2)
                    right.setPosition(pos - lashFix);
                if (power3)
                    middle.setPosition(pos);
                if (power4)
                    other.setPosition(pos);
            }
            else {
                left.setPosition(t1);
                right.setPosition(t2);
                middle.setPosition(t3);
                other.setPosition(t4);
            }
            telemetry.addData("left", left.getPosition());
            telemetry.addData("right", right.getPosition());
            telemetry.addData("middle", middle.getPosition());
        }
    }



    public void periodic() {
        //if (Robot.logData) log();
        aim();
        sotm = Launcher.teleop;
        if (continuousMode) {
            spin.calculate();
            current = -getTotalDegrees();
            controller.update(current, target);
            llcontroller.updateConstants(p2, d2, f2, l2, i2);

            power = controller.run();

            power = Range.clip(power, -1, 1);


            spin.setPower(power);
            spin2.setPower(power);

            telemetry.addData("turret power", power);
            telemetry.addData("turret volts", spin.getVolts());
        }
        else {
            double b = getPos();
            if (!testLash) {
                left.setPosition(b + lashFix);
                right.setPosition(b - lashFix);
                middle.setPosition(b);
                other.setPosition(b);
            }
            else {
                left.setPosition(t1);
                right.setPosition(t2);
                middle.setPosition(t3);
                other.setPosition(t4);
            }
        }

        if (showTelemetry)
            telemetry.addData("turret target", target);
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
        target = targetDeg + lashOffset;
        turretOffAuto = true;
    }

    public void updateAiming(double targetX, double targetY, Pose botPose) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.botPose = botPose;
    }

    public double getPos() {
        return zeroPos + (target / 180.0) * (oneEightyPos - zeroPos);
    }



    public void aim() {
        /*
         * Calculates the turret angle  relative to the robot's front (degrees).
         * Clamps to [-90°, +90°].
         */

//        double vxTemp = 0;//KinematicsCalculator.inchesToMeters(r.getFollower().getVelocity().getXComponent());
//        double vx = Double.isNaN(vxTemp) ? 0 : vxTemp;
//        double vyTemp = 0;//KinematicsCalculator.inchesToMeters(r.getFollower().getVelocity().getYComponent());
//        double vy = Double.isNaN(vyTemp) ? 0 : vyTemp;
//        double va = 0;//r.getFollower().getAngularVelocity();

        if (sotm) {
            vx = Constants.localizer.getVelocity().getX();
            vy = Constants.localizer.getVelocity().getY();
            dvx = vx - prevX;
            dvy = vy - prevY;
            prevX = vx;
            prevY = vy;
            vx += dvx * velWeight;
            vy += dvy * velWeight;
        } else {
            vx = 0;
            vy = 0;
            dvx = 0;
            dvy = 0;
        }

        double dx;
        double dy;

        double x = botPose.getX();
        double y = botPose.getY();
        dx = targetX - x - vx;
        dy = targetY - y - vy;
        double robotHeading = Math.toDegrees(botPose.getHeading());

        double angleToTargetField = Math.toDegrees(Math.atan2(dy, dx));
        double turretRelativeAngle;

        turretRelativeAngle = wrapTo180(angleToTargetField - robotHeading + fudgeFactor) ;

        turretRelativeAngle = Range.clip(turretRelativeAngle, MIN_ANGLE, MAX_ANGLE);
        //turretRelativeAngle = 0;
        if (useTurret) {
            if (Launcher.teleop || !turretOffAuto) {
                if (continuousMode)
                    setTargetDegrees(turretRelativeAngle);
                else
                    setTargetDegrees(turretRelativeAngle);
            }
        }
        else
            target = 0;

        telemetry.addData("Target Degrees", continuousMode ? -turretRelativeAngle : turretRelativeAngle);

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
    public double getVx() {
        return vx;
    }
    public double getVy() {
        return vy;
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