package org.firstinspires.ftc.teamcode.opmode.automus;

import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.FarAuto.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.config.vision.Vision;
import org.firstinspires.ftc.teamcode.config.vision.VisionBall;

import java.util.ArrayList;
import java.util.Comparator;

@Autonomous(name = "Far")
@Config
//@Configurable
public class FarRemade extends OpMode {
    //private MultipleTelemetry telemetry;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot robot;
    int done = 0;
    boolean two = false;
    double onThreshold = .05;
    double onThresholdTwo = 0.4;
    // 6767 - Julian
    double offThreshold = 0.05;
    double moveThreshold = 2;
    double moveThresholdTwo = 3.5;
    double moveIntakeThreshold = 1;

    double ballCVWaitThreshold = .4;
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

    public Double ballY;




    public static boolean sotm = false;
    public static boolean twentyOne = false;
    public static double tValue = .3;
    public boolean dontChangeTurret = false;
    private Vision vision;


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
                robot.limelight.setPipeline(1);
                break;

            case 10:
                followPath(robot.getAlliance() == Alliance.RED ? shoot1(robot.getFollower()) : shoot1Blue(robot.getFollower()), true);
                if (shootPath(true))
                    setPathState(12025);
                break;

            case 12025:
                followPath(robot.getAlliance() == Alliance.RED ? spikeMarkPickup(robot.getFollower()) : spikeMarkPickupBlue(robot.getFollower()), true);
                if (intakePath(false))
                    setPathState(1204);
                break;

            case 1204:
                followPath(robot.getAlliance() == Alliance.RED ? shootSpikeMark(robot.getFollower()) : shootSpikeMarkBlue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(1301);
                break;

            case 1301:
                followPath(robot.getAlliance() == Alliance.RED ? humanPlayerZone(robot.getFollower()) : humanPlayerZoneBlue(robot.getFollower()), true);
                if (intakePath(false))
                    setPathState(1302);
                break;

            case 1302:
                followPath(robot.getAlliance() == Alliance.RED ? shootHumanPlayer(robot.getFollower()) : shootHumanPlayerBlue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(14);
                break;

            case 14:
                Double y = getBallY();
                if (y != null || pathTimer.getElapsedTimeSeconds() > ballCVWaitThreshold) {
                    ballY = y;
                    setPathState(12);
                }
                else {
                    ballY = humanPlayer.getY();
                }

            case 12:
                if (ballY == null)
                    followPath(robot.getAlliance() == Alliance.RED ? humanPlayerZone(robot.getFollower()) : humanPlayerZoneBlue(robot.getFollower()), true);
                else 
                    followPath(robot.getAlliance() == Alliance.RED ? ballCVIntake(robot.getFollower(), ballY) : humanPlayerZoneBlue(robot.getFollower()), true);
                if (intakePath(false))
                    setPathState(13);
                break;



            case 13:
                followPath(robot.getAlliance() == Alliance.RED ? ballCVShoot(robot.getFollower(), robot.getFollower().getPose()) : humanPlayerZoneBlue(robot.getFollower()), true);
                if (shootPath())
                    setPathState(14);
                break;
        /*

            case 1452:
                followPath(robot.getAlliance() == Alliance.RED ? gatePickup(robot.getFollower()) : gatePickup2Blue(robot.getFollower()), true);
                if (intakePath(false))
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
                break; */

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
        //Robot.shootPose = shootPose2;
        pathTimer = new Timer();
    }

    @Override
    public void start() {
        vision = new Vision();
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

    public boolean shotDone(boolean b) {
        if (robot.intake.getUptakeState() == Intake.UptakeState.OFF) {
            //pathTimer.reset();
            return false;
        }
        //if (!two)
        if (pathTimer.getElapsedTimeSeconds() < checkTime + onThreshold) return false;
        //else
        //  if (pathTimer.getElapsedTimeSeconds() < checkTime + onThresholdTwo + .15) return false;
        if (pathTimer.getElapsedTimeSeconds() > moveThresholdTwo || /*(robot.intake.uptake.getCurrent(CurrentUnit.AMPS) < 1.1 && robot.intake.intake.getCurrent(CurrentUnit.AMPS) < 1.8) || */robot.intake.none()) {
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

    private boolean intakePath(boolean gatePath) {
        robot.intake.setGateState(Intake.GateState.CLOSED);
        if (robot.getFollower().getCurrentTValue() < tValue) {
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
        if (pathDone()) {
            if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                if (robot.launcher.atTarget()) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                }
                else {
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                }
                robot.shotStarted = true;
            }

            if (doneDone) {
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
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
            if (robot.getFollower().getCurrentTValue() > 1 - tValue) {
                robot.intake.setIntakeState(Intake.IntakeState.OFF);
                robot.intake.setUptakeState(Intake.UptakeState.OFF);
                robot.intake.setGateState(Intake.GateState.OPEN);
                robot.launcher.setLauncherState(Launcher.LauncherState.SHOOT);
            }
        }
        return false;
    }

    private boolean shootPath(boolean b) {
        if (pathDone()) {
            if (pathTimer.getElapsedTimeSeconds() > onThreshold) {
                if (robot.launcher.atTarget()) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKE);
                    robot.intake.setUptakeState(Intake.UptakeState.ON);
                }
                else {
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                }
                robot.shotStarted = true;
            }

            if (doneDone) {
                if (pathTimer.getElapsedTimeSeconds() > offThreshold) {
                    robot.intake.setIntakeState(Intake.IntakeState.OFF);
                    robot.intake.setUptakeState(Intake.UptakeState.OFF);
                    robot.shotStarted = false;
                    doneDone = false;
                    first = true;
                    return true;
                }
            }
            if (shotDone(true)) {
                pathTimer.reset();
                doneDone = true;
                time = pathTimer.getElapsedTimeSeconds() > moveThreshold;
            }
        }
        else {
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

    private Double getBallY() {
        Pose robotPose = robot.getFollower().getPose();
        vision.processCurrentBalls(robot.limelight.getDetectorResult(), robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        ArrayList<VisionBall> ballsInMemory = vision.getCachedBalls();
        ballsInMemory.sort(Comparator.comparingDouble(VisionBall::getDistance));
        if (!ballsInMemory.isEmpty()) {
            if (ballsInMemory.size() > 1)
                return ballsInMemory.get(1).getAbsoluteY();
            else return ballsInMemory.get(0).getAbsoluteY();
        }
        else return null;
    }
}


