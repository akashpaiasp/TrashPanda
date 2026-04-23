package org.firstinspires.ftc.teamcode.opmode.automus;

import static org.firstinspires.ftc.teamcode.config.core.Robot.intakeThreshold;
import static org.firstinspires.ftc.teamcode.config.core.Robot.uptakeThreshold;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.FarAuto.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.Timer;

@Disabled
@Autonomous (name = "Far")
@Config
//@Configurable
public class Far extends OpMode {
    //private MultipleTelemetry telemetry;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;
    boolean two = false;
    double onThreshold = 0.1;
    double firstShootThreshold = .4;

    double offThreshold = 0.3;
    double moveThreshold = 3.8;
    double moveIntakeThreshold = .4;
    public static boolean firstCouple = true;
    double doneNum = 0;
    double intakeTime = 2;
    double checkTime = 0;
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

    public boolean humanPlayer = false;

    public boolean  shot1Done = false;



    public void autonomousPathUpdate() {

        if (aimTurret || true) {
            robot.turret.turretOffAuto = false;
        }//new Aim(robot, goalX, goalY).execute();
        else if (!dontChangeTurret){
            if (robot.getAlliance() == Alliance.RED)
                robot.turret.setTargetDegrees(0);
            else
                robot.turret.setTargetDegrees(0);
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
                robot.launcher.setLauncherState(Launcher.LauncherState.SHOOT);
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




            case 1201:
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        //setPathState(1202);
                        setPathState(1207);
                        doneDone = false;

                    }
                }
                else {

                if ((robot.getFollower().isBusy())) {
                    pathTimer.reset();
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() < onThreshold) {
                    return;
                }
                if(robot.validLaunch) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                }
                else {
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                }
                    robot.shotStarted = true;


                if (shotDone()) {
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                }
                //stopProgram = true;
                break;


            case 1202:
                doneNum = 0;
                firstCouple = false;
                ////robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? humanPlayerZone(robot.getFollower()) : humanPlayerZoneBlue(robot.getFollower()), false);
                robot.intake.setGateState(Intake.GateState.CLOSED);
                sotm = false;
                //if (gamepad1.square)
                setPathState(12025);
                break;

            case 12025:
                if (robot.getFollower().getCurrentTValue() > .3) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                }
                if (robot.getFollower().isBusy()){
                    pathTimer.reset();
                    return;
                }
                else {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shootHumanPlayer(robot.getFollower()) : shootHumanPlayerBlue(robot.getFollower()));
                    two = true;
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(1205);
                }

                break;

            case 1205:
                if (robot.getFollower().getCurrentTValue() > .4) {
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    setPathState(1207);
                }
            case 1207:
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? spikeMarkPickup(robot.getFollower()) : spikeMarkPickupBlue(robot.getFollower()));
                        setPathState(13);

                        doneDone = false;
                    }
                }
                else {
                    if (robot.getFollower().isBusy()) {
                        pathTimer.reset();
                        return;
                    }


                    if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                        if (robot.validLaunch) {
                            robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                            robot.intake.setUptakeState(Intake.UptakeState.ON);
                        } else {
                            robot.intake.setIntakeState(Intake.IntakeState.OFF);
                            robot.intake.setUptakeState(Intake.UptakeState.OFF);
                        }

                        robot.shotStarted = true;
                    }

                    if (shotDone()) {

                        pathTimer.reset();
                        doneDone = true;
                        time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                        //if (gamepad1.square)
                    }
                }
                break;


            case 13:
                if (robot.getFollower().getCurrentTValue() > .1) {
                    robot.intake.setGateState(Intake.GateState.CLOSED);
                }
                if (robot.getFollower().getCurrentTValue() > .5) {

                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                }
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (intakeDone())
                {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shootSpikeMark(robot.getFollower()) : shootSpikeMarkBlue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    setPathState(14);
                }
                break;

            case 14:
                if (robot.getFollower().getCurrentTValue() > .5) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.SHOOT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    setPathState(145);
                }
                break;
            case 145:
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        setPathState(1452);
                        doneDone = false;

                    }
                }

                else {
                    if (robot.getFollower().isBusy()) {
                        pathTimer.reset();
                        return;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                        if (robot.validLaunch) {
                            robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                            robot.intake.setUptakeState(Intake.UptakeState.ON);
                        } else {
                            robot.intake.setIntakeState(Intake.IntakeState.OFF);
                            robot.intake.setUptakeState(Intake.UptakeState.OFF);
                        }
                        robot.shotStarted = true;
                    }
                    if (shotDone()) {
                        pathTimer.reset();
                        doneDone = true;
                        time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                        //if (gamepad1.square)
                    }
                }
                break;

            case 1452: {
                doneNum = 0;
                //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? humanPlayerZone(robot.getFollower()) : humanPlayerZoneBlue(robot.getFollower()), true);
                robot.intake.setGateState(Intake.GateState.CLOSED);
                //if (gamepad1.square)
                setPathState(1453);

            }

            break;





            case 1453:
                if (robot.getFollower().getCurrentTValue() > .3) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.SLOW);
                }
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                if (intakeDone())
                {
                    if (getRuntime() < 26) {
                        robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shootHumanPlayer(robot.getFollower()) : shootHumanPlayerBlue(robot.getFollower()), true);
                        aim1 = false;
                        robot.intake.setIntakeState(Intake.IntakeState.OFF);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                        setPathState(1454);
                    }
                    else {
                        robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? park(robot.getFollower()) : parkBlue(robot.getFollower()), true);

                        robot.intake.setIntakeState(Intake.IntakeState.OFF);
                        robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    }
                }
                break;

            case 1454:
                if (robot.getFollower().getCurrentTValue() > 0.5) {
                    robot.launcher.setLauncherState(Launcher.LauncherState.SHOOT);
                    robot.intake.setGateState(Intake.GateState.OPEN);
                    aimTurret = true;
                    setPathState(1455);
                }
                break;
            case 1455:
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
                        if (humanPlayer)
                            setPathState(1452);
                        else
                            setPathState(1452);
                        humanPlayer = !humanPlayer;
                        doneDone = false;
                    }
                }
                else {
                    if (robot.getFollower().isBusy()) {
                        pathTimer.reset();
                        return;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                        if (robot.validLaunch) {
                            robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                            robot.intake.setUptakeState(Intake.UptakeState.ON);
                        } else {
                            robot.intake.setIntakeState(Intake.IntakeState.OFF);
                            robot.intake.setUptakeState(Intake.UptakeState.OFF);
                        }
                        robot.shotStarted = true;
                    }
                    if (shotDone()) {

                        pathTimer.reset();
                        doneDone = true;
                        time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                        //if (gamepad1.square)
                    }
                }
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
        resetRuntime();
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
        return robot.intake.none() || pathTimer.getElapsedTimeSeconds() > moveThreshold;
    }
    public boolean intakeDone() {
        if (pathTimer.getElapsedTimeSeconds() < checkTime) return false;
        return pathTimer.getElapsedTimeSeconds() > moveIntakeThreshold || robot.intake.has3();
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


