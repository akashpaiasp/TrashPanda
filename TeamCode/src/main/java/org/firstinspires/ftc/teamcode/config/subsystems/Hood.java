package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config

public class Hood extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    public Servo hoodR, hoodL;

    public static double shootingVariable = 0.02;
    public static double hoodDown = .35; //23.9
    public double hoodMid = 0.4 ;
    public double hoodMidUp = 0.5; //64(hoodPos-0.5) + 23.9 //6.4
    public static double hoodUp = .77; //49.5
    public static double target = 0.0;
    public static double hoodIncreaseAmt = 0.02;
    public static double autoHoodShoot1 = 0.9;
    public static double autoHoodShoot2 = .9;

    public enum HoodState {
        UP,
        MIDUP,
        MID,
        DOWN,
        MANUAL
    }
    public HoodState current = HoodState.MANUAL ;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        hoodR = hardwareMap.get(Servo.class, "sh15");
        hoodL = hardwareMap.get(Servo.class, "sh10");
        target = 0.0;
    }

    public void setState(HoodState state) {
        current = state;
    }

    public void init() {
        setState(HoodState.UP);
    }


    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {

        telemetry.addData("Hood", hoodR.getPosition());
        telemetry.addData("Hood state", current);
        switch (current) {
            case UP:
                target = hoodUp;
                //hood.setPosition(hoodUp);
                break;
            case MIDUP:
                target = hoodMidUp;
                //hood.setPosition(hoodMidUp);
                break;
            case MID:
                target = hoodMid;
                //hood.setPosition(hoodMid);
                break;
            case DOWN:
                target = hoodDown;
                //hood.setPosition(hoodDown);
                break;
            case MANUAL :
                hoodR.setPosition(target);
                hoodL.setPosition(target);

        }
        hoodR.setPosition(target);
        hoodL.setPosition(target);
    }
    public void increase() {
        setState(HoodState.MANUAL);
        target += .05;
    }
    public void decrease() {
        setState(HoodState.MANUAL);
        target -= .05;
    }

    public void increaseSmall() {
        setState(HoodState.MANUAL);
        target += hoodIncreaseAmt;
    }
    public void decreaseSmall() {
        setState(HoodState.MANUAL);
        target -= hoodIncreaseAmt;
    }
    public HoodState getState() {
        return current;
    }
    public double getTarget() {
        return target;
    }
    public void setTarget(double t) {
        setState(HoodState.MANUAL);
        target = Range.clip(t, hoodDown, hoodUp);
    }
}