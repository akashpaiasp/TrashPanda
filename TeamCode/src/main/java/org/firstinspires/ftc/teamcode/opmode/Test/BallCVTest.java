package org.firstinspires.ftc.teamcode.opmode.Test;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;
import static org.firstinspires.ftc.teamcode.config.core.Robot.p;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.config.vision.Vision;
import org.firstinspires.ftc.teamcode.config.vision.VisionBall;

import java.util.ArrayList;

@TeleOp
public class BallCVTest extends LinearOpMode {
    private Robot robot;
    private Vision vision;
    public static Pose pose1, pose2, pose3;
    public void runOpMode() throws InterruptedException {
        //Initialize Hardware

        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, telemetry, Alliance.RED, autoEndPose);
        vision = new Vision();
        robot.init();
        robot.getFollower().setStartingPose(p);
        waitForStart();
        robot.tStart();
        robot.limelight.setPipeline(1);

        while(opModeIsActive()) {

            robot.getFollower().update();
            robot.getFollower().setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.robotCentric
            );
            robot.intake.periodic();
            Pose robotPose = robot.getFollower().getPose();
            vision.processCurrentBalls(robot.limelight.getDetectorResult(), robotPose.getX(), robotPose.getY(), robotPose.getHeading());
            ArrayList<VisionBall> balls = vision.getCachedBalls();
            telemetry.addData("Robot x", robotPose.getX());
            telemetry.addData("Robot y", robotPose.getY());
            telemetry.addLine();
            for (VisionBall ball : balls) {
                telemetry.addData("Absolute x", ball.getAbsoluteX());
                telemetry.addData("Absolute y", ball.getAbsoluteY());
                telemetry.addData("Relative x", ball.getRelativeX());
                telemetry.addData("Relative y", ball.getRelativeY());
                telemetry.addLine();
            }
            telemetry.update();



            if (gamepad1.a) {
                robot.intake.setGateState(Intake.GateState.CLOSED);
                robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                robot.getFollower().followPath(vision.getBallPath(robot.getFollower()));
            }
            else if (!robot.getFollower().isBusy()) {
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
            }
            if (gamepad1.left_bumper) {
                robot.getFollower().startTeleopDrive();
            }


        }
        //CSVInterface.log();
    }
}