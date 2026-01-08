package org.firstinspires.ftc.teamcode.opmode.automus;

import static org.firstinspires.ftc.teamcode.config.core.Robot.alliance;
import static org.firstinspires.ftc.teamcode.config.core.Robot.blueX;
import static org.firstinspires.ftc.teamcode.config.core.Robot.goalY;
import static org.firstinspires.ftc.teamcode.config.core.Robot.redX;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.EighteenBall.*;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class EighteenBall extends OpMode {
    //private MultipleTelemetry telemetry;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;
    boolean two = false;
    double onThreshold = .5;
    double onThresholdTwo = .5;

    double offThreshold = .2;
    double moveThreshold = 3.5;
    double moveIntakeThreshold = 1.8;
    boolean doneOff = false;
    double doneNum = 0;
    double outtakeTIme = .15;
    double lastMoveTime = 5.2;
    double intakeTime = 2;
    double checkTIme = .31;
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


    public void autonomousPathUpdate() {

        if (aimTurret) new Aim(robot, alliance == Alliance.RED ? redX : blueX, goalY).execute();
        else robot.turret.setTargetDegrees(-62);
        //if (aim1) robot.turret.setTargetDegrees(-62);
        //else robot.turret.setTargetDegrees(-45);

        robot.aPeriodic();

        switch (pathState) {
            case 00: //preload & set max power
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
                if (robot.getFollower().getCurrentTValue() > .8) {
                    //aimTurret = true;
                    setPathState(1201);
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
                    if(robot.validLaunch) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON); }
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
                    //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe1(robot.getFollower()) : strafe1Blue(robot.getFollower()), true);
                    robot.intake.setGateState(Intake.GateState.CLOSED);
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
                else  {
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot2(robot.getFollower()) : shoot2Blue(robot.getFollower()), true);
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    //if (gamepad1.square)
                        setPathState(1205);
                }
                break;

            case 1205:
                if (robot.getFollower().getCurrentTValue() > 0) {
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
                    if(robot.validLaunch) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON); }
                    robot.shotStarted = true;
                }
                if (doneDone) {
                    if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                        robot.shotStarted = false;
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


                //end


            case 12: {
                two = false;
                doneNum = 0;
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
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
                    if(robot.validLaunch) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
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
                robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
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
                    if(robot.validLaunch) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
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
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
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
                    if(robot.validLaunch) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
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
                    robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? pickup2(robot.getFollower()) : strafe1Blue(robot.getFollower()), true);
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
                    robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? shoot3(robot.getFollower()) : shoot3Blue(robot.getFollower()), true);
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
                    if(robot.validLaunch) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
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
                robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.getFollower().followPath(robot.getAlliance() == Alliance.RED ? strafe2(robot.getFollower()) : strafe2Blue(robot.getFollower()), true);
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
                    setPathState(273);
                }
                break;
            case 273:
                if (robot.getFollower().isBusy()) {
                    pathTimer.reset();
                    return;
                }
                
                if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                    if(robot.validLaunch) {robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
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
                    pathTimer.reset();
                    doneDone = true;
                    time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
                    //if (gamepad1.square)
                }
                break;

            case 275:
                if (!robot.getFollower().isBusy()) {
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
            robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
            robot.intake.setIntakeState(Intake.IntakeState.OFF);
            robot.intake.setUptakeState(Intake.UptakeState.OFF);
        }
        pathTimer.reset();
    }

    @Override
    public void init () {
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
        if (!two)
            if (pathTimer.getElapsedTimeSeconds() < checkTIme + onThreshold) return false;
        else
            if (pathTimer.getElapsedTimeSeconds() < checkTIme + onThresholdTwo) return false;
        if (pathTimer.getElapsedTimeSeconds() > moveThreshold || (robot.intake.uptake.getCurrent(CurrentUnit.AMPS) < 1 && robot.intake.intake.getCurrent(CurrentUnit.AMPS) < 3)) {
            aimTurret = false;
            return true;
        }
        else return false;
    }
    public boolean intakeDone() {
        if (pathTimer.getElapsedTimeSeconds() < checkTIme) return false;
        return pathTimer.getElapsedTimeSeconds() > moveIntakeThreshold || (robot.intake.uptake.getCurrent(CurrentUnit.AMPS) > 4 && robot.intake.intake.getCurrent(CurrentUnit.AMPS) > 1.8);
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


