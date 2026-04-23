package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;


@Config
public class Intake extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;
    private Servo gate;
    public DcMotorEx intake, uptake;
    public DigitalChannel bb1, bb2, bb3;
    public RevColorSensorV3 bottom, middle, top;
    //larger number = further from shooter
    public static boolean useColor = true;

    public static double launchIntake = 1;
    public static double launchUptake = 1;
    public static double intakeUptake = 1;
    public static double outtake1Power = -.7;

    public static boolean manual = false;

    public static double gatePos = 0.5;

    public static boolean autoOuttake = true;
    private static double
            open = .5,
            closed = .75;

    public enum IntakeState {
        OUTTAKE,
        INTAKE,
        OFF,
        SLOWOUTTAKE

    }

    public enum UptakeState {
        ON,
        OFF,
        SLOW,
        BACK,
        SLOWOUTTAKE
    }
    public enum GateState {
        OPEN,
        CLOSED
    }
    public IntakeState currentIntake = IntakeState.OFF;
    public UptakeState currentUptake = UptakeState.OFF;
    public GateState currentGate = GateState.CLOSED;


    //state of the subsystem


    // public DcMotorEx

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //pusherL = hardwareMap.get(Servo.class, "cs1");
        //pusherM = hardwareMap.get(Servo.class, "cs2");
        //pusherM = hardwareMap.get(Servo.class, "cs3");

        gate = hardwareMap.get(Servo.class, "cs3");

        intake = hardwareMap.get(DcMotorEx.class, "cm2");
        uptake = hardwareMap.get(DcMotorEx.class, "em2");


        bb1 = hardwareMap.get(DigitalChannel.class, "cd1");
        bb2 = hardwareMap.get(DigitalChannel.class, "ed1");
        bb3 = hardwareMap.get(DigitalChannel.class, "ed6");

        middle = hardwareMap.get(RevColorSensorV3.class, "ci3");
        top = hardwareMap.get(RevColorSensorV3.class, "ci2");
        bottom = hardwareMap.get(RevColorSensorV3.class, "ei1");


        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        uptake.setDirection(DcMotorSimple.Direction.FORWARD);

        //init servos based on their name in the robot's config file

    }

    //Call this method to open/close the servos


    //methods to change the state

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */

    public void setIntakeState(IntakeState intakeState) {
        if (autoOuttake) {
            if ((intakeState == IntakeState.INTAKE || intakeState == IntakeState.OFF) && has3()) {
                currentIntake = IntakeState.SLOWOUTTAKE;
            }
            else currentIntake = intakeState;
        }
        else
            currentIntake = intakeState;
    }
    public void setUptakeState(UptakeState uptakeState) {
        currentUptake = uptakeState;
    }
    public void setGateState(GateState gateState) {
        currentGate = gateState;
    }
    public void periodic() {
        switch (currentIntake) {
            case OFF:
                intake.setPower(0);
                break;
            case INTAKE:
                intake.setPower(launchIntake);
                break;
            case OUTTAKE:
                intake.setPower(-1);
                break;
            case SLOWOUTTAKE:
                intake.setPower(outtake1Power);
                break;
        }

        switch (currentUptake) {
            case OFF:
                uptake.setPower(0);
                break;
            case ON:
                uptake.setPower(launchUptake);
                //setGateState(GateState.OPEN);
                break;
            case SLOW:
                uptake.setPower(intakeUptake);
                //setGateState(GateState.CLOSED);
                break;
            case BACK:
                uptake.setPower(-1);
                break;
            case SLOWOUTTAKE:
                uptake.setPower(outtake1Power);
        }
        if(manual) {
            gate.setPosition(gatePos);
        }
        else {
            switch (currentGate) {

                case OPEN:
                    gate.setPosition(open);
                    break;
                case CLOSED:
                    gate.setPosition(closed);
                    break;
            }
        }

        if (showTelemetry) {
            telemetry.addData("Intake amps", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Uptake amps", uptake.getCurrent(CurrentUnit.AMPS));

            if (!useColor) {
                telemetry.addData("bb1", bb1.getState());
                telemetry.addData("bb2", bb2.getState());
                telemetry.addData("bb3", bb3.getState());
            }
            else {
                telemetry.addData("bb1", bb1.getState());
                telemetry.addData("bottom in", bottom.getDistance(DistanceUnit.INCH));
                telemetry.addData("bottom color", toString(bottom));
                telemetry.addData("middle in", middle.getDistance(DistanceUnit.INCH));
                telemetry.addData("middle color", toString(middle));
                telemetry.addData("top in", top.getDistance(DistanceUnit.INCH));
                telemetry.addData("top color", toString(top));
            }
        }
    }
    public String toString(RevColorSensorV3 b) {
        return "R: " + b.red() + ", G:" + b.green() + ", B: " + b.blue() + ", A: " + b.alpha();
    }

    public boolean has3() {
        return false && (!bb1.getState() && !bb2.getState() && !bb3.getState());
    }

    public boolean none() {
        return bb1.getState() && bb2.getState() && bb3.getState() && false;
    }

    public int num() {
        if (true)
            return 0;
        if (!bb1.getState()) {
            if (!bb2.getState()) {
                if (!bb3.getState())
                    return 3;
                else
                    return 2;
            }
            else if (!bb3.getState())
                return 2;
            else return 1;
        }
        else if (!bb2.getState()) {
                if (!bb3.getState())
                    return 2;
                else
                    return 1;
            }
        else if (!bb3.getState()) {
            return 1;
        }
        else return 0;

    }

    public void init() {
        setIntakeState(IntakeState.OFF);
        intake.setPower(0);
        setGateState(GateState.CLOSED);
        gate.setPosition(closed);
    }

    public void log(){
        Logger.logData(LogType.INTAKE_POWER, String.valueOf(intake.getPower()));
    }

    public UptakeState getUptakeState() {
        return currentUptake;
    }


}