package org.firstinspires.ftc.teamcode.opmode.Test;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;

import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Hood;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.logging.CSVInterface;

@TeleOp
public class HoodTest extends LinearOpMode {
    //hi
    private Hood h;
    private GamepadEx g1;
    private GamepadEx g2;

    public void runOpMode() throws InterruptedException {
        //Initialize Hardware
        h = new Hood(hardwareMap, telemetry);
        waitForStart();

        while(opModeIsActive()) {
            h.periodic();
        }
        CSVInterface.log();
    }
}
