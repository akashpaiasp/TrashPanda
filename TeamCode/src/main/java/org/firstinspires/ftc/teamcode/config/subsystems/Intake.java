package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;


@Config
public class Intake extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;
    private Servo gateL, gateR;
    public DcMotorEx intake, uptake;
    public static double launchIntake = 1;
    public static double launchUptake = 1;
    public static double intakeUptake = .7;

    public static boolean manual = false;

    public static double gateLPos = 0.5;
    public static double gateRPos = 0.5;

    private static double
            lOpen = .6,
            lClosed = .35,
            rOpen = .42,
            rClosed = 0.64;

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
        BACK
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

        gateL = hardwareMap.get(Servo.class, "sh5");
        gateR = hardwareMap.get(Servo.class, "sh4");
        intake = hardwareMap.get(DcMotorEx.class, "cm0");
        uptake = hardwareMap.get(DcMotorEx.class, "cm1");


        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        uptake.setDirection(DcMotorSimple.Direction.REVERSE);

        //init servos based on their name in the robot's config file

    }

    //Call this method to open/close the servos


    //methods to change the state

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */

    public void setIntakeState(IntakeState intakeState) {
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
                intake.setPower(-.25);
                break;
        }

        switch (currentUptake) {
            case OFF:
                uptake.setPower(0);
                break;
            case ON:
                uptake.setPower(launchUptake);
                break;
            case SLOW:
                uptake.setPower(intakeUptake);
                break;
            case BACK:
                uptake.setPower(-1);
        }
        if(manual) {
            gateL.setPosition(gateLPos);
            gateR.setPosition(gateRPos);
        }
        else {
            switch (currentGate) {

                case OPEN:
                    gateL.setPosition(lOpen);
                    gateR.setPosition(rOpen);
                    break;
                case CLOSED:
                    gateL.setPosition(lClosed);
                    gateR.setPosition(rClosed);
                    break;
            }
        }

        telemetry.addData("Intake amps", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Uptake amps", uptake.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("Intake state", currentIntake);
        //telemetry.addData("Uptake state", currentUptake);
    }

    public void init() {
        setIntakeState(IntakeState.OFF);
        intake.setPower(0);
        setGateState(GateState.CLOSED);
        gateL.setPosition(lClosed);
        gateR.setPosition(rClosed);
    }

    public void log(){
        Logger.logData(LogType.INTAKE_POWER, String.valueOf(intake.getPower()));
    }

    public UptakeState getUptakeState() {
        return currentUptake;
    }
}