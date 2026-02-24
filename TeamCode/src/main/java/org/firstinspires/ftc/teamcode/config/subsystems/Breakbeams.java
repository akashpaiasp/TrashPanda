package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/
@Config

public class Breakbeams extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    //state of the subsystem
    public DigitalChannel bb1, bb2, bb3;
    //larger number = further from shooter

    public enum BallState {
        ball,
        noBall
    }

    public Breakbeams(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init servos based on their name in the robot's config file
        bb1 = hardwareMap.get(DigitalChannel.class, "ed0");
        bb2 = hardwareMap.get(DigitalChannel.class, "ed1");
        bb3 = hardwareMap.get(DigitalChannel.class, "ed6");
    }


    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodic() {

        telemetry.addData("bb1", bb1.getState());
        telemetry.addData("bb2", bb2.getState());
        telemetry.addData("bb3", bb3.getState());
    }
}