package org.firstinspires.ftc.teamcode.opmode.automus;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.gatePickup;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.gatePickup2Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.gatePickupBlue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.pickup1Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.pickup2Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.pickup3Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.pickupFirstSpike;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.secondSpike;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot1;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot1Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot2;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot2Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot3;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot3Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot4;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shoot4Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shootGate;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shootGate2Blue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shootGateBlue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.shootPose2;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.startPose;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.startPoseBlue;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.TwentyOneBall.thirdSpike;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.Timer;

@Autonomous(name = "fake 24")
@Config
//@Configurable
public class TwentyFour extends OpMode {
    //private MultipleTelemetry telemetry;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;
    boolean two = false;
    double onThreshold = 0;
    double onThresholdTwo = 0.4;
    // 6767 - Julian
    double offThreshold = 0;
    double moveThreshold = .5;
    double moveIntakeThreshold = 1;
    public static boolean firstCouple = true;
    boolean doneOff = false;
    double doneNum = 0;
    double checkTime = 0;
    double gateOpenTimeFirstIntake = .1;
    boolean time = false;
    boolean doneDone = false;
    boolean aimTurret = false;
    boolean stopProgram = false;
    double currTime = 0;
    boolean aim1 = true;
    boolean pressingBack = false;
    boolean first = true;


    public static boolean sotm = false;
    public static boolean twentyOne = false;
    public static double tValue = .3;
    public static double shootTValue = .86;
    public boolean dontChangeTurret = false;


    public void autonomousPathUpdate() {

        if (aimTurret || true) {
            robot.turret.turretOffAuto = false;
        }//new Aim(robot, goalX, goalY).execute();
        else if (!dontChangeTurret) {
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
                robot.getFollower().setMaxPower(1);
                setPathState(10);
                robot.launcher.setLauncherState(Launcher.LauncherState.SHOOT);
                break;

            case 10:
                followPath(robot.getAlliance() == Alliance.RED ? shoot1(robot.getFollower()) : shoot1Blue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(12025);
                break;

            case 12025:
                followPath(robot.getAlliance() == Alliance.RED ? secondSpike(robot.getFollower()) : pickup1Blue(robot.getFollower()), true);
                if (intakePath(false))
                    setPathState(1204);
                break;

            case 1204:
                followPath(robot.getAlliance() == Alliance.RED ? shoot2(robot.getFollower()) : shoot2Blue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(1301);
                break;

            case 1301:
                followPath(robot.getAlliance() == Alliance.RED ? gatePickup(robot.getFollower()) : gatePickupBlue(robot.getFollower()), true);
                if (intakePath(true))
                    setPathState(1302);
                break;

            case 1302:
                followPath(robot.getAlliance() == Alliance.RED ? shootGate(robot.getFollower()) : shootGateBlue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(12);
                break;

            case 12:
                followPath(robot.getAlliance() == Alliance.RED ? gatePickup(robot.getFollower()) : gatePickupBlue(robot.getFollower()), true);
                if (intakePath(true))
                    setPathState(13);
                break;


            case 13:
                followPath(robot.getAlliance() == Alliance.RED ? shootGate(robot.getFollower()) : shootGateBlue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(1452);
                break;


            case 1452:
                followPath(robot.getAlliance() == Alliance.RED ? gatePickup(robot.getFollower()) : gatePickup2Blue(robot.getFollower()), true);
                if (intakePath(true))
                    setPathState(1453);
                break;


            case 1453:
                followPath(robot.getAlliance() == Alliance.RED ? shootGate(robot.getFollower()) : shootGate2Blue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(1456);
                break;

            case 1456:
                followPath(robot.getAlliance() == Alliance.RED ? pickupFirstSpike(robot.getFollower()) : pickup2Blue(robot.getFollower()), true);
                if (intakePath(false))
                    setPathState(19);
                break;

            case 19:
                followPath(robot.getAlliance() == Alliance.RED ? shoot3(robot.getFollower()) : shoot3Blue(robot.getFollower()), false);
                if (shootPath())
                    setPathState(24);
                break;

            case 24:
                followPath(robot.getAlliance() == Alliance.RED ? thirdSpike(robot.getFollower()) : pickup3Blue(robot.getFollower()), true);
                if (intakePath(false))
                    setPathState(271);
                break;


            case 271:
                followPath(robot.getAlliance() == Alliance.RED ? shoot4(robot.getFollower()) : shoot4Blue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(28);
                break;



            case 28:
                setPathState(28);
                break;

            case 99999:
                break;

        }


    }

    public void setPathState(int pState) {
        if (!stopProgram) {
            pathState = pState;
            currTime = getRuntime();

        } else {
            pathState = 99999;
            //robot.launcher.setLauncherState(Launcher.LauncherState.STOP);
            robot.intake.setIntakeState(Intake.IntakeState.OFF);
            robot.intake.setUptakeState(Intake.UptakeState.OFF);
        }
        pathTimer.reset();
    }

    @Override
    public void init() {
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
        } else {
            robot.getFollower().setStartingPose(startPoseBlue);
        }
    }

    @Override
    public void loop() {
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
        } else if (!gamepad1.back)
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
        if (pathTimer.getElapsedTimeSeconds() > moveThreshold || /*(robot.intake.uptake.getCurrent(CurrentUnit.AMPS) < 1.1 && robot.intake.intake.getCurrent(CurrentUnit.AMPS) < 1.8) || */robot.intake.none()) {
            aimTurret = false;
            return true;
        } else return false;
    }

    public boolean intakeDone() {
        return pathTimer.getElapsedTimeSeconds() > moveIntakeThreshold || robot.intake.has3();
    }

    private boolean pathDone() {
        if (robot.getFollower().isBusy()) {
            pathTimer.reset();
            return false;
        }
        return true;
    }
    private boolean pathDone(boolean shotPath) {
        if (robot.getFollower().getCurrentTValue() < shootTValue && robot.getFollower().isBusy()) {
            pathTimer.reset();
            return false;
        }
        return true;
    }

    private boolean intakePath(boolean gatePath) {
        robot.intake.setGateState(Intake.GateState.CLOSED);
        if (robot.getFollower().getCurrentTValue() < tValue && robot.getFollower().isBusy()) {
            if (robot.getFollower().getCurrentTValue() > 1-shootTValue) {
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
            }
            return false;
        }
        robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
        robot.intake.setUptakeState(Intake.UptakeState.SLOW);
        if (gatePath) {
            if (pathDone() && intakeDone()) {
                first = true;
                return true;
            }
        } else {
            if (robot.intake.has3() || pathDone()) {
                first = true;
                return true;
            }
        }
        return false;
    }

    private boolean shootPath() {
        if (pathDone(true)) {
            telemetry.addData("Path done", true);
            if (pathTimer.getElapsedTimeSeconds() >= onThreshold) {
                robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                robot.intake.setUptakeState(Intake.UptakeState.ON);
                robot.shotStarted = true;
            }

            if (doneDone) {
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.shotStarted = false;
                    doneDone = false;
                    first = true;
                    return true;
                }
            }
            if (shotDone()) {
                pathTimer.reset();
                doneDone = true;
                time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
            }
        }
        else {
            telemetry.addData("Path done", false);
            if (robot.getFollower().getCurrentTValue() > 1 - tValue) {
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.intake.setGateState(Intake.GateState.OPEN);
                robot.launcher.setLauncherState(Launcher.LauncherState.SHOOT);
            }
        }
        return false;
    }

    private void followPath(PathChain p, boolean b) {
        if (first)
            robot.getFollower().followPath(p, b);
        first = false;
    }
}


