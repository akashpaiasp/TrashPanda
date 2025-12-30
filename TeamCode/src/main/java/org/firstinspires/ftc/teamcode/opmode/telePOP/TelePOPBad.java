package org.firstinspires.ftc.teamcode.opmode.telePOP;

import static org.firstinspires.ftc.teamcode.config.core.Robot.alliance;
import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;
import static org.firstinspires.ftc.teamcode.config.core.Robot.blueX;
import static org.firstinspires.ftc.teamcode.config.core.Robot.goalY;
import static org.firstinspires.ftc.teamcode.config.core.Robot.redX;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.config.commands.Aim;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.Timer;
import org.firstinspires.ftc.teamcode.config.util.logging.CSVInterface;

@Config
@Disabled
@TeleOp (name = "Bad teleop")

public class TelePOPBad extends LinearOpMode {
    private Robot robot;
    private GamepadEx g1;
    private GamepadEx g2;
    private double scaleFactor = 1;
    public Timer loopTimer = new Timer();
    private double lastTime = 0, currentTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        //Initialize Hardware
        robot = new Robot(hardwareMap, telemetry, Robot.alliance, autoEndPose);
        Robot.auto = false;

        //Initialize Gamepads
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        //Reset instance


        waitForStart();
        loopTimer.reset();
        robot.tStart();
        robot.dualControls(g1, g2);


        while (opModeIsActive()) {
            lastTime = currentTime;
            currentTime = loopTimer.getElapsedTime();
            telemetry.addData("Loop Time", currentTime - lastTime);
            telemetry.addData("Distance From Goal", robot.getDistanceFromGoal());
            telemetry.addData("Shot Num", robot.shotNum);

            //Update everything
            robot.tPeriodic();


            if (gamepad1.right_trigger > 0.3) {
                robot.uptakeOff = false;
                robot.intakeOff = false;
                robot.intake.setGateState(Intake.GateState.CLOSED);
                robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                robot.intake.setUptakeState(Intake.UptakeState.SLOW);
            }
            else if (gamepad1.left_trigger > 0.3) {
                //robot.uptakeOff = true;
                robot.intakeOff = false;
                robot.uptakeOff = false;
                robot.intake.setIntakeState(Intake.IntakeState.OUTTAKE);
                robot.intake.setUptakeState(Intake.UptakeState.BACK);
            }
            else {
                if (robot.intakeOff)
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                if (robot.uptakeOff)
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
            }


            if(!gamepad1.right_bumper && ! (gamepad1.right_trigger > 0.3) & ! (gamepad1.left_trigger > 0.3)) {
                robot.intakeOff = true;
                robot.uptakeOff = true;
            }

            new Aim(robot, alliance == Alliance.RED ? redX : blueX, goalY).execute();
            //}

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                robot.intake.setGateState(Intake.GateState.OPEN);
            }
            else {
                robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
            }



            //Runs all gamepad triggers
            CommandScheduler.getInstance().run();




            //Driving (driver 1)
            //if (!gamepad2.right_bumper) {
            robot.getFollower().setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.robotCentric
            );
            // } else {
            // robot.getFollower().setTeleOpDrive(0, 0, 0, 0);
            //  }


            //Use this driving code only if pedro pathing doesn't work
            /*
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad


            scaleFactor = 1;
            scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), .4);

            // Send calculated power to wheels
                robot.driveTrain.lf.setPower(leftFrontPower * scaleFactor);
                robot.driveTrain.rf.setPower(rightFrontPower * scaleFactor);
                robot.driveTrain.lr.setPower(leftBackPower * scaleFactor);
                robot.driveTrain.rr.setPower(rightBackPower * scaleFactor); */
            telemetry.addData("x" , robot.getFollower().getPose().getX());
            telemetry.addData("y" , robot.getFollower().getPose().getY());
            telemetry.addData("heading" , robot.getFollower().getPose().getHeading());

        }
        CSVInterface.log();
    }

}