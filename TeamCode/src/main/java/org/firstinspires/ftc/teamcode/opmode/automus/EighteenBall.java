package org.firstinspires.ftc.teamcode.opmode.automus;

import static org.firstinspires.ftc.teamcode.config.core.Robot.alliance;
import static org.firstinspires.ftc.teamcode.config.core.Robot.blueX;
import static org.firstinspires.ftc.teamcode.config.core.Robot.goalX;
import static org.firstinspires.ftc.teamcode.config.core.Robot.goalY;
import static org.firstinspires.ftc.teamcode.config.core.Robot.intakeThreshold;
import static org.firstinspires.ftc.teamcode.config.core.Robot.redX;
import static org.firstinspires.ftc.teamcode.config.core.Robot.uptakeThreshold;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.EighteenBall.*;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.commands.Aim;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.Timer;


@Autonomous (name = "Big Smapple")
@Config
//@Configurable
public class EighteenBall extends OpMode {
    //private MultipleTelemetry telemetry;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;
    boolean two = false;
    double onThreshold = 0;
    double onThresholdTwo = 0;

    double offThreshold = 0;
    double moveThreshold = 1.8;
    double moveIntakeThreshold = 1.2;
    public static boolean firstCouple = true;
    boolean doneOff = false;
    double doneNum = 0;
    double outtakeTIme = .15;
    double lastMoveTime = 5.2;
    double intakeTime = 2;
    double checkTime = .15;
    double gateOpenTimeFirstIntake = .1;
    int doneThreshold = 4;
    double dist;
    boolean time = false;
    boolean doneDone = false;
    boolean aimTurret = false;
    boolean firstAim = true;
    boolean timeupdate = true;
    double p = 0;
    boolean stopProgram = false;
    double currTime = 0;
    boolean aim1 = true;
    boolean pressingBack = false;


    public static double rpm = 4000;
    public static boolean sotm = false;
    public static boolean twentyOne = false;
    public static double hood1 = .7;
    public static double hood2 = .8;
    public static double hood3 = .9;
    public static double timeBetween = .5;
    public static double tValue = .3;
    public boolean dontChangeTurret = false;



    public void autonomousPathUpdate() {

        if (aimTurret) {
            robot.turret.turretOffAuto = false;
        }//new Aim(robot, goalX, goalY).execute();
        else if (!dontChangeTurret){
            if (robot.getAlliance() == Alliance.RED)
                robot.turret.setTargetDegrees(-62);
            else
                robot.turret.setTargetDegrees(62);
        }
        //if (aim1) robot.turret.setTargetDegrees(-62);
        //else robot.turret.setTargetDegrees(-45);

        robot.aPeriodic();

        switch (pathState) {
            case 00: //preload & set max power
                //sotm = true;
                robot.getFollower().setMaxPower(1);
                //robot.turret.setTargetDegrees(robot.getAlliance() == Alliance.RED ? -53 : 53);
                //if (gamepad1.square)
                setPathState(10);
                break;


            case 10:
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ?  shoot1(robot.getFollower()) : shoot1Blue(robot.getFollower()),  true);
                robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                robot.intake.setGateState(Intake.GateState.OPEN);
                //if (gamepad1.square)
                setPathState(1025);
                break;
            case 1025:
                if (robot.getFollower().getCurrentTValue() >= 0) {
                        aimTurret = true;
                        //sotm = true;
                        //robot.launcher.setTarget(rpm);
                        // setPathState(1026);
                    setPathState(1201);
                }
                break;

            case 1026:
                if (robot.getFollower().getCurrentTValue() >= tValue) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                    robot.shotStarted = true;

                    if (pathTimer.getElapsedTimeSeconds() > timeBetween) {
                        if (pathTimer.getElapsedTimeSeconds() > timeBetween * 2) {
                            robot.hood.setTarget(hood3);
                            setPathState(1201);
                        }
                        else
                            robot.hood.setTarget(hood2);
                    }
                    else
                        robot.hood.setTarget(hood1);
                }
                break;

//            case 11:
//                if (robot.getFollower().isBusy()) {
//                    pathTimer.reset();
//                    return;
//                }
//                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
//                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
//                    robot.intake.setUptakeState(Intake.UptakeState.ON);
//                }
//                //if (gamepad1.square)
//                    setPathState(1201);
//
//                break;

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
            //start



            case 1201:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        setPathState(1202);
                        doneDone = false;

                    }
                }
                if (shotDone()) {
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                //stopProgram = true;
                break;


            case 1202:
                doneNum = 0;
                firstCouple = false;
                ////robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe1(robot.getFollower()) : strafe1Blue(robot.getFollower()), false);
                robot.intake.setGateState(Intake.GateState.CLOSED);
                sotm = false;
                //if (gamepad1.square)
                setPathState(12025);
                break;

            case 12025:
                if (robot.getFollower().isBusy()){
                    pathTimer.reset();
                    return;
                }
                else {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup1(robot.getFollower()) : pickup1Blue(robot.getFollower()));
                    two = true;
                    setPathState(1204);
                }

                break;
            case 1204:
                robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                if (robot.getFollower().isBusy() ){
                    pathTimer.reset();
                    return;
                }
                else if (pathTimer.getElapsedTimeSeconds() < gateOpenTimeFirstIntake) return;
                else  {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot2(robot.getFollower()) : shoot2Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    //if (gamepad1.square)
                    setPathState(1205);
                }
                break;

            case 1205:
                if (robot.getFollower().getCurrentTValue() > 0.2) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    //if (gamepad1.square)
                    setPathState(1207);
                }
                break;
            case 1207:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }


                if (pathTimer.getElapsedTimeSeconds() > onThresholdTwo) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);

                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        if (!twentyOne)setPathState(12);
                        else setPathState(1301);
                        doneDone = false;
                    }
                }
                if (shotDone()) {
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                break;

                //21 ball here
            case 1301: {
                two = false;
                doneNum = 0;
                ////robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? gatePickup(robot.getFollower()) : gatePickupBlue(robot.getFollower()), true);
                robot.intake.setGateState(Intake.GateState.CLOSED);
                //if (gamepad1.square)
                setPathState(1302);

            }
            break;





            case 1302:
                if (robot.getFollower().getCurrentTValue() > .9) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                }
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (intakeDone())
                {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shootGate(robot.getFollower()) : shootGateBlue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(1303);
                }
                break;

            case 1303:
                if (robot.getFollower().getCurrentTValue() > 0) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    setPathState(1304);
                }
                break;
            case 1304:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    if(true) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON); }
                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        //setPathState(1452);
                        setPathState(12);
                        doneDone = false;

                    }
                }
                if (shotDone()) {
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                break;


            //21 ball end


            case 12: {
                two = false;
                doneNum = 0;
                ////robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? gatePickup(robot.getFollower()) : gatePickupBlue(robot.getFollower()), true);
                robot.intake.setGateState(Intake.GateState.CLOSED);
                //if (gamepad1.square)
                setPathState(13);

            }
            break;





            case 13:
                if (robot.getFollower().getCurrentTValue() > .9) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                }
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (intakeDone())
                {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shootGate(robot.getFollower()) : shootGateBlue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(14);
                }
                break;

            case 14:
                if (robot.getFollower().getCurrentTValue() > 0) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    setPathState(145);
                }
                break;
            case 145:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    if(true) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON); }
                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        setPathState(1452);
                        doneDone = false;

                    }
                }
                if (shotDone()) {
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                break;

            case 1452: {
                doneNum = 0;
                //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? gatePickup2(robot.getFollower()) : gatePickup2Blue(robot.getFollower()), true);
                robot.intake.setGateState(Intake.GateState.CLOSED);
                //if (gamepad1.square)
                setPathState(1453);

            }
            break;





            case 1453:
                if (robot.getFollower().getCurrentTValue() > .9) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                }
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (intakeDone())
                {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shootGate2(robot.getFollower()) : shootGate2Blue(robot.getFollower()), true);
                    aim1 = false;
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(1454);
                }
                break;

            case 1454:
                if (robot.getFollower().getCurrentTValue() > 0) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    setPathState(1455);
                }
                break;
            case 1455:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    if(true) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON); }
                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        setPathState(1456);
                        doneDone = false;
                    }
                }
                if (shotDone()) {
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                break;

            //here
            case 1456:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    //robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    //robot.intake.setUptakeState(Intake.UptakeState.OFF);

                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup2(robot.getFollower()) : pickup2Blue(robot.getFollower()), true);
                    robot.intake.setGateState(Intake.GateState.CLOSED);
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);

                    setPathState(19);
                }
                break;
    /*

            case 1452:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (pathTimer.getElapsedTimeSeconds() > intakeTime) {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shootGate(robot.getFollower()) : shoot2Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(1453);
                }
                break;

            case 1453:
                if (robot.getFollower().getCurrentTValue() > 0.2) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    setPathState(1454);
                }
                break;
            case 1454:
                if (!robot.getFollower().isBusy()) {
                    setPathState(1455);
                    doneOff = true;
                }
                break;
            case 1455:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    if(true) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON); }
                }
                if (pathTimer.getElapsedTimeSeconds() > moveThreshold)
                    setPathState(19);
                break;

*/

            /*
            case 19:
                doneNum = 0;
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup2(robot.getFollower()) : strafe1Blue(robot.getFollower()), false);
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                    robot.intake.setGateState(Intake.GateState.CLOSED);
                    setPathState(1925);
                }

                break;

               */
            case 19:
                aim1 = true;
                if (robot.getFollower().isBusy()  ){
                    pathTimer.reset();
                    return;
                }
                else  {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot3(robot.getFollower()) : shoot3Blue(robot.getFollower()), false);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    //if (gamepad1.square)
                    setPathState(20);
                }
                break;

            case 20:
                if (robot.getFollower().getCurrentTValue() > 0) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    //if (gamepad1.square)
                    setPathState(22);
                }
                break;
            //here

            case 22:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    if(true) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON); }
                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        setPathState(23);
                        doneDone = false;
                    }
                }
                if (shotDone()) {
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                break;

            //now

            case 23:
                doneNum = 0;
                //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe2(robot.getFollower()) : strafe2Blue(robot.getFollower()), false);
                robot.intake.setGateState(Intake.GateState.CLOSED);
                //if (gamepad1.square)
                setPathState(24);
                break;

            case 24:
                if (robot.getFollower().isBusy()){
                    pathTimer.reset();
                    return;
                }
                else {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup3(robot.getFollower()) : pickup3Blue(robot.getFollower()));
                    setPathState(271);
                }

                break;


            case 271:
                if (robot.getFollower().isBusy() ) {
                    pathTimer.reset();
                    return;
                }
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot4(robot.getFollower()) : shoot4Blue(robot.getFollower()), true);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                setPathState(272);
                break;

            case 272:
                if (robot.getFollower().getCurrentTValue() > 0) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.OUT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    two = true;
                    setPathState(273);
                }
                break;
            case 273:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() > onThresholdTwo) {
                    if(true) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                        robot.intake.setUptakeState(Intake.UptakeState.ON); }
                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        setPathState(275);
                        doneDone = false;
                    }
                }
                if (shotDone()) {
                    dontChangeTurret = true;
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                break;

            case 275:
                if (!robot.getFollower().isBusy()) {
                    //if (robot.getAlliance() == Alliance.BLUE)
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? move(robot.getFollower()) : moveBlue(robot.getFollower()), true);
                    setPathState(28);
                }

            case 28:
                setPathState(28);
                break;

            case 99999:
                break;

        }


    }

    public void setPathState ( int pState){
        if (!stopProgram) {
            pathState = pState;
            currTime = getRuntime();

        }
        else {
            pathState = 99999;
            //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
            robot.intake.setIntakeState(Intake.IntakeState.OFF);
            robot.intake.setUptakeState(Intake.UptakeState.OFF);
        }
        pathTimer.reset();
    }

    @Override
    public void init () {
        firstCouple = true;
        CommandScheduler.getInstance().reset();
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry, Alliance.RED, startPose);
        Robot.shootPose = shootPose2;
        pathTimer = new Timer();
    }
    @Override
    public void start() {
        if (robot.getAlliance() == Alliance.RED) {
            robot.getFollower().setStartingPose(startPose);
        }
        else {
            robot.getFollower().setStartingPose(startPoseBlue);
        }
    }

    @Override
    public void loop () {
        if (gamepad1.square)
            stopProgram = true;
        //robot.getFollower().update();
        autonomousPathUpdate();
        CommandScheduler.getInstance().run();


        robot.getTelemetry().addData("Path State", pathState);
        robot.getTelemetry().addData("Position", robot.getFollower().getPose().toString());

        robot.getTelemetry().addData("Time", currTime);
        robot.getTelemetry().addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        robot.getTelemetry().addData("Uptake Current", robot.intake.uptake.getCurrent(CurrentUnit.AMPS));
        robot.getTelemetry().addData("Intake Current", robot.intake.intake.getCurrent(CurrentUnit.AMPS));
        robot.getTelemetry().addData("Used time", time);
        //robot.getTelemetry().update();
        robot.auto = true;
    }

    @Override
    public void init_loop() {
        robot.getTelemetry().addData("pose", robot.getFollower().getPose());
        robot.aInitLoop(new GamepadEx(gamepad1));
        if (gamepad1.back && !pressingBack) {
            robot.setAlliance(Alliance.BLUE);
            pressingBack = true;
        }
        else if (!gamepad1.back)
            pressingBack = false;

    }

    public boolean shotDone() {
        if (robot.intake.getUptakeState() == Intake.UptakeState.OFF) {
            //pathTimer.reset();
            return false;
        }
        //if (!two)
        if (pathTimer.getElapsedTimeSeconds() < checkTime + onThreshold) return false;
        //else
        //  if (pathTimer.getElapsedTimeSeconds() < checkTime + onThresholdTwo + .15) return false;
        if (pathTimer.getElapsedTimeSeconds() > moveThreshold || (robot.intake.uptake.getCurrent(CurrentUnit.AMPS) < .8 && robot.intake.intake.getCurrent(CurrentUnit.AMPS) < 1.8)) {
            aimTurret = false;
            return true;
        }
        else return false;
    }
    public boolean intakeDone() {
        if (pathTimer.getElapsedTimeSeconds() < checkTime) return false;
        return pathTimer.getElapsedTimeSeconds() > moveIntakeThreshold || (robot.intake.uptake.getCurrent(CurrentUnit.AMPS) > uptakeThreshold && robot.intake.intake.getCurrent(CurrentUnit.AMPS) > intakeThreshold);
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


