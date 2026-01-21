package org.firstinspires.ftc.teamcode.opmode.automus;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwelveBall.*;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Hood;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.Timer;


@Autonomous
@Config
//@Configurable
public class TwelveBallNoGate extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;
    double offThreshold = 0.2;
    double moveThreshold = 2.2;
    boolean doneOff = false;
    double doneNum = 0;
    int doneThreshold = 4;


    public void autonomousPathUpdate() {

        robot.aPeriodic();

        switch (pathState) {
            case 00: //preload & set max power
                robot.getFollower().setMaxPower(1);
                robot.turret.setTargetDegrees(robot.getAlliance() == Alliance.RED ? 45 : -45);
                robot.hood.setState(Hood.HoodState.UP);
                setPathState(10);
                break;


            case 10:
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ?  shoot1(robot.getFollower()) : shoot1Blue(robot.getFollower()), true);
                robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                setPathState(105);
                break;
            case 105:
                if (!robot.getFollower().isBusy()) {
                    setPathState(11);
                    doneOff = true;
                }
                break;

            case 11:
                if (!robot.getFollower().isBusy()) {
                    if (robot.launcher.controller.done) {
                        doneOff = false;
                        robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON);
                    }
                    else {
                        if (!doneOff) {
                            doneOff = true;
                            doneNum++;
                        }
                        robot.intake.setIntakeState(Intake.IntakeState.OFF);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                        setPathState(12);
                }

                break;

            /*case 115:
                if (!robot.launcher.controller.done || pathTimer.getElapsedTimeSeconds() > 1) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.intake.setIntakeState(Intake.IntakeState.STOP);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                        setPathState(116);
                    }
                }
                break;

            case 116:
                if (robot.launcher.controller.done || pathTimer.getElapsedTimeSeconds() > 1) {
                    if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                        robot.intake.setIntakeState(Intake.IntakeState.SLOW);
                        robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                        setPathState(117);
                    }
                }
                break;

            case 117:
                if (!robot.launcher.controller.done || pathTimer.getElapsedTimeSeconds() > 1) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.intake.setIntakeState(Intake.IntakeState.STOP);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                        setPathState(118);
                    }
                }
                break;

            case 118:
                if (robot.launcher.controller.done || pathTimer.getElapsedTimeSeconds() > 1) {
                    if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                        robot.intake.setIntakeState(Intake.IntakeState.SLOW);
                        robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                        setPathState(12);
                    }
                }
                break;
             */


            case 12: {
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup1(robot.getFollower()) : pickup1Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);

                    setPathState(13);
                }

            }



            break;

            /*

            case 125:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? gate(robot.getFollower()) : gateBlue(robot.getFollower()), true);
                    setPathState(13);
                }
                break; */

            case 13:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot2(robot.getFollower()) : shoot2Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(14);

                }
                break;

            case 14:
                if (robot.getFollower().getCurrentTValue() > 0.7) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    setPathState(1425);
                }
                break;
            case 1425:
                if (!robot.getFollower().isBusy()) {
                    setPathState(145);
                    doneOff = true;
                }
                break;
            case 145:
                if (!robot.getFollower().isBusy()) {
                    if (robot.launcher.controller.done) {
                        doneOff = false;
                        robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON);
                    }
                    else {
                        if (!doneOff) {
                            doneOff = true;
                            doneNum++;
                        }
                        robot.intake.setIntakeState(Intake.IntakeState.OFF);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                        setPathState(19);
                }
                break;


            case 19:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe1(robot.getFollower()) : strafe1Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);

                    setPathState(195);
                }

                break;
            case 195:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup2(robot.getFollower()) : pickup2Blue(robot.getFollower()), true);
                    setPathState(20);
                }
                break;

            case 20:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot3(robot.getFollower()) : shoot3Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(21);

                }
                break;

            case 21:
                if (robot.getFollower().getCurrentTValue() > 0.7) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    setPathState(215);
                }
                break;
            case 215:
                if (!robot.getFollower().isBusy()) {
                    setPathState(22);
                    doneOff = true;
                }
                break;
            case 22:
                if (!robot.getFollower().isBusy()) {
                    if (robot.launcher.controller.done) {
                        doneOff = false;
                        robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON);
                    }
                    else {
                        if (!doneOff) {
                            doneOff = true;
                            doneNum++;
                        }
                        robot.intake.setIntakeState(Intake.IntakeState.OFF);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                        setPathState(27);
                }
                break;


            case 27:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe2(robot.getFollower()) : strafe2Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);

                    setPathState(275);
                }

                break;
            case 275:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup3(robot.getFollower()) : pickup3Blue(robot.getFollower()), true);
                    setPathState(28);
                }

            case 28:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ?shoot4(robot.getFollower()) : shoot4Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(29);

                }
                break;

            case 29:
                if (robot.getFollower().getCurrentTValue() > 0.7) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    setPathState(295);
                }
                break;
            case 295:
                if (!robot.getFollower().isBusy()) {
                    setPathState(30);
                    doneOff = true;
                }
                break;
            case 30:
                if (!robot.getFollower().isBusy()) {
                    if (robot.launcher.controller.done) {
                        doneOff = false;
                        robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON);
                    }
                    else {
                        if (!doneOff) {
                            doneOff = true;
                            doneNum++;
                        }
                        robot.intake.setIntakeState(Intake.IntakeState.OFF);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                        setPathState(35);
                }
                break;



            case 35:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ?move(robot.getFollower()) : moveBlue(robot.getFollower()), true);
                    robot.turret.setTargetDegrees(0);
                    setPathState(36);
                }
                break;

            case 36:
                break;


        }


    }

    public void setPathState ( int pState){
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void init () {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED, startPose);
        pathTimer = new Timer();
    }

    @Override
    public void loop () {
        robot.getFollower().update();
        autonomousPathUpdate();


        robot.getTelemetry().addData("Path State", pathState);
        robot.getTelemetry().addData("Position", robot.getFollower().getPose().toString());
        robot.getTelemetry().update();
        Robot.auto = true;
    }

    @Override
    public void init_loop() {
        robot.aInitLoop(new GamepadEx(gamepad1));
        if (gamepad1.back) {
            robot.setAlliance(Alliance.BLUE);
            robot.getFollower().setStartingPose(convertToBlue(startPose));
        }
    }

    public void launch3() {

        boolean finished = false;
        if (!robot.launcher.controller.done) {

            finished = false;
        } else {
            if (!finished) {
                finished = true;
                done++;
            }
            robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
            robot.intake.setUptakeState(Intake.UptakeState.ON);
        }
    }
}


