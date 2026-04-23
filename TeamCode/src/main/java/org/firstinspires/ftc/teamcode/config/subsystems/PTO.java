package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

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

public class PTO extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    public Servo pto;
    public static boolean manualTarget = false;
    public static double manualPTOTarget = .5;
    public static double off = .65;
    public static double on = 0.75;
    public static double target = 0.0;

    public enum PTOState {
        OFF,
        ON
    }
    public PTOState current = PTOState.OFF;

    public PTO(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        pto = hardwareMap.get(Servo.class, "es3");
        target = .5;
    }

    public void setState(PTOState state) {
        current = state;
    }

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {
        if (showTelemetry) {
            telemetry.addData("Hood", pto.getPosition());
            telemetry.addData("Hood state", current);
        }
        switch (current) {
            case OFF:
                target = off;
                break;
            case ON:
                target = on;
                break;
        }
        if (!manualTarget)
            pto.setPosition(target);
        else
            pto.setPosition(manualPTOTarget);
    }

}