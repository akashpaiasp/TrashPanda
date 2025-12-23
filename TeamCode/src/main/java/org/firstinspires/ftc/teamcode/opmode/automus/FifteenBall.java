package org.firstinspires.ftc.teamcode.opmode.automus;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwelveBall.*;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class FifteenBall extends OpMode {
    public static double shootHood = 0.9;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;
    double onThreshold = 0;
    double offThreshold = 0.4;
    double moveThreshold = 1.25;
    boolean doneOff = false;
    double doneNum = 0;
    double outtakeTIme = 0;
    double lastMoveTime = 5.2;
    int doneThreshold = 4;


    public void autonomousPathUpdate() {

        robot.aPeriodic();

        switch (pathState) {
            case 00: //preload & set max power
                robot.getFollower().setMaxPower(1);
                robot.turret.setTargetDegrees(robot.getAlliance() == Alliance.RED ? -53 : 53);
                robot.hood.setTarget(Hood.autoHoodShoot2);
                setPathState(10);
                break;


            case 10:
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ?  shoot1(robot.getFollower()) : shoot1Blue(robot.getFollower()), true);
                robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                robot.intake.setGateState(Intake.GateState.OPEN);
                setPathState(105);
                break;
            case 105:
                if (!robot.getFollower().isBusy()) {
                    setPathState(11);
                    doneOff = true;
                }
                break;

            case 11:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                    if (robot.launcher.shotDetected && !robot.shotFired) {
                        robot.shotFired = true;
                        robot.hood.decreaseSmall();
                    } else if (!robot.launcher.shotDetected) {
                        robot.shotFired = false;
                    }
                }

                    /*
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
                    } */
                if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                    setPathState(12);

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
                    robot.hood.setTarget(Hood.autoHoodShoot2);
                    robot.turret.setTargetDegrees(robot.getAlliance() == Alliance.RED ? -48 : 48);
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup1(robot.getFollower()) : pickup1Blue(robot.getFollower()), true);
                    robot.intake.setGateState(Intake.GateState.CLOSED);
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
                break;*/

            case 13:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot215(robot.getFollower()) : shoot215Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(14);

                }
                break;

            case 14:
                if (robot.getFollower().getCurrentTValue() > 0.2) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
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
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                    if (robot.launcher.shotDetected && !robot.shotFired) {
                        robot.shotFired = true;
                        robot.hood.decreaseSmall();
                    } else if (!robot.launcher.shotDetected) {
                        robot.shotFired = false;
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                    setPathState(19);
                break;


            case 19:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe1(robot.getFollower()) : strafe1Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                    robot.intake.setGateState(Intake.GateState.CLOSED);
                    robot.hood.setTarget(Hood.autoHoodShoot2);

                    setPathState(195);
                }

                break;
            case 195:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup2(robot.getFollower()) : pickup2Blue(robot.getFollower()), true);
                    setPathState(1925);
                }
                break;
            case 1925:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? gate15(robot.getFollower()) : gate15Blue(robot.getFollower()), true);
                    setPathState(1975);
                }
            case 1975:
                if (!robot.getFollower().isBusy() || pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(20);
                }
                break;

            case 20:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot315(robot.getFollower()) : shoot315Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(21);

                }
                break;

            case 21:
                if (robot.getFollower().getCurrentTValue() > 0.2) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
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
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                    if (robot.launcher.shotDetected && !robot.shotFired) {
                        robot.shotFired = true;
                        robot.hood.decreaseSmall();
                    } else if (!robot.launcher.shotDetected) {
                        robot.shotFired = false;
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                    setPathState(27);
                break;


            case 27:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.hood.setTarget(Hood.autoHoodShoot2);
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe2(robot.getFollower()) : strafe2Blue(robot.getFollower()), true);
                    robot.intake.setGateState(Intake.GateState.CLOSED);
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
                if (robot.getFollower().getCurrentTValue() > 0.2) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
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
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                    if (robot.launcher.shotDetected && !robot.shotFired) {
                        robot.shotFired = true;
                        robot.hood.decreaseSmall();
                    } else if (!robot.launcher.shotDetected) {
                        robot.shotFired = false;
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                    setPathState(35);
                break;



            case 35:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.hood.setTarget(Hood.autoHoodShoot2);
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                    robot.intake.setGateState(Intake.GateState.CLOSED);
                    //robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ?move(robot.getFollower()) : moveBlue(robot.getFollower()), true);
                    setPathState(36);
                }
                break; //end of 12 ball


            case 36:
                if (!robot.getFollower().isBusy()) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup4(robot.getFollower()) : pickup4Blue(robot.getFollower()), true);
                    setPathState(37);
                }

            case 37:
                if (!robot.getFollower().isBusy() || pathTimer.getElapsedTimeSeconds() > lastMoveTime) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ?shoot5(robot.getFollower()) : shoot5Blue(robot.getFollower()), true);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    setPathState(375);

                }
                break;


            case 375:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    robot.intake.setIntakeState(Intake.IntakeState.SLOWOUTTAKE);
                    pathTimer.reset();
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    if (pathTimer.getElapsedTimeSeconds() > outtakeTIme) {
                        robot.intake.setIntakeState(Intake.IntakeState.OFF);
                        setPathState(38);
                    }
                }

            case 38:
                if (robot.getFollower().getCurrentTValue() > 0.2) {


                    setPathState(39);
                }
                break;
            case 39:
                if (!robot.getFollower().isBusy()) {
                    setPathState(40);
                    doneOff = true;
                }
                break;
            case 40:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                    if (robot.launcher.shotDetected && !robot.shotFired) {
                        robot.shotFired = true;
                        robot.hood.decreaseSmall();
                    } else if (!robot.launcher.shotDetected) {
                        robot.shotFired = false;
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                    setPathState(41);
                break;



            case 41:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.hood.setTarget(Hood.autoHoodShoot2);
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? move(robot.getFollower()) : moveBlue(robot.getFollower()), true);
                    setPathState(42);
                    robot.turret.setTargetDegrees(0);
                }
                break;

            case 42:
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
        robot.auto = true;
    }

    @Override
    public void init_loop() {
        //robot.aInitLoop(new GamepadEx(gamepad1));
        telemetry.addData("Alliance", robot.getAlliance());
        telemetry.addData("Start Pose X", robot.getFollower().getPose().getX());
        telemetry.addData("Start Pose Y", robot.getFollower().getPose().getY());
        telemetry.addData("Start Pose X", robot.getFollower().getHeading());
        if (gamepad1.back) {
            robot.setAlliance(Alliance.BLUE);
            robot.getFollower().setPose(startPoseBlue);
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
