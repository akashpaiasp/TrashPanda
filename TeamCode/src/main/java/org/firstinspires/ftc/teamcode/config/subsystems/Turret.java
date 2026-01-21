package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.Robot;
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

    public static double offset = -140;
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
    public double current;
    public boolean limelightMode = false;


    private MultipleTelemetry telemetry;
    public AxonContinuous spin;
    public CRServo spin2;
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
        current = -(Math.round(getTotalDegrees() * 10.0)) / 10.0;
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
        spin.calculate();
        current = -(Math.round(getTotalDegrees() * 10.0)) / 10.0;
        controller.update(current, target);
        llcontroller.updateConstants(p2, d2, f2, l2, i2);

        power = controller.run();

        power = Range.clip(power, -1, 1);

        spin.setPower(power);
        spin2.setPower(power);

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
    }

    public void log() {
        Logger.logData(LogType.TURRET_TARGET, String.valueOf(target));
        Logger.logData(LogType.TURRET_VOLTS, String.valueOf(spin.getVolts()));
        Logger.logData(LogType.TURRET_PREV, String.valueOf(spin.lastVoltage));
        Logger.logData(LogType.TURRET_FULL_ROTS, String.valueOf(spin.full_rotations));

    }

    public void updateLL(double d) {
        llcontroller.update(d, 1);
    }

}