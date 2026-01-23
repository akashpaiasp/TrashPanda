package org.firstinspires.ftc.teamcode.config.core;

import static org.firstinspires.ftc.teamcode.config.core.util.Opmode.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.config.commands.*;
import org.firstinspires.ftc.teamcode.config.core.paths.AutoDriving;
import org.firstinspires.ftc.teamcode.config.core.util.*;
import org.firstinspires.ftc.teamcode.config.util.KinematicsCalculator;
import org.firstinspires.ftc.teamcode.config.util.PoseEkf;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.*;
import org.firstinspires.ftc.teamcode.config.util.Timer;
import org.firstinspires.ftc.teamcode.opmode.automus.EighteenBall;

import java.util.List;


@Config
public class Robot {
    private HardwareMap hw;
    //private MultipleTelemetry telemetry;
    private Telemetry telemetry;
    private Follower follower;
    private Opmode op = TELEOP;
    private double speed = 1.0;
    public static double turretOffset = 3.8;
    public static double r = 1;

    //Booleans to change on FTCDash
    public static boolean showTelemetry = false;
    public static boolean hoodAdjustment = false;
    public static boolean rapidFireFar = true;
    public static boolean autoShoot = false;
    public static boolean keepShooterOn = true;

    public static double robot_length = 8.5; //actual robot length is 9, decreasing it means more robot has to be in zone in order to shoot

    public double robotX = 0, robotY = 0;
    public boolean rev = false;


    public AutoDriving autoDrive;
    //90 - x : -3.8, y : 0
    //0 - x : 0, y : -3.8
    //-90 - x : -3.8, y : 0
    //180 - x : 0, y : 3.8
    public static Pose p = new Pose(0, 0, Math.toRadians(90));

    public static Pose autoEndPose = p.copy();
    public Launcher launcher;
    public Turret turret;
    public Hood hood;
    public LED led;

    public KinematicsCalculator k;
    public boolean slowMode;
    public Limelight limelight;
    public Intake intake;
    public boolean robotCentric = false;
    public static Alliance alliance = Alliance.RED;
    public static Zone zone = Zone.NONE;



    public static double centerX = 67, centerY = 67, centerXBlue = -67;
    public static double centerX2 = centerX, centerY2 = centerY;
    public static double goalY = centerY;
    public static double redX = centerX;
    public static double blueX = centerXBlue;
    public static double goalX = centerX;

    public static boolean manualR = false;
    public static boolean manualRPM = false;
    public boolean shotStarted = false;
    public static Pose shootPose;

    public static boolean robotCentricDrive = false;




    //double centerX = redX, centerY = goalY;
    double rightWallX = centerX - 7, rightWallY = centerY;  // right wall center
    double frontWallX = centerX, frontWallY = centerY - 7;


    double maxDist = 72;
    public static double goalDist = 52;
    public static double farLaunchDist = 100;

    //public static Pose cornerBlueFront = new Pose(-72, -72);
    public static Pose cornerBlueBack = new Pose(-61.9, -65.9);
    // public static Pose cornerRedFront = new Pose(-72, -72);
    public static Pose cornerRedBack = new Pose(61.9, -65.9);

    public boolean uptakeOff = true;
    public boolean launcherOff = true;
    public boolean intakeOff = true;

    public static boolean auto = false;

    public static boolean logData = false;

    public boolean shotFired = false;
    public boolean justShot = false;

    public int shotNum = -1;
    public boolean rBumper = false;
    public boolean outtake = true;
    public boolean validLaunch = false;
    public  double turretX;
    public  double turretY;
    public static double flightTime = .5;
    int i = 3;




    public int flip = 1, tState = -1, sState = -1, spec0State = -1, spec180State = -1, c0State = -1, aFGState = -1, specTransferState = -1, fSAState = -1, sRState = -1, hState = -1;
    private boolean aInitLoop, frontScore = false, backScore = true, automationActive = false;

    public static double
            processNoiseXY = 0.7, processNoiseHeading = 0.03,
            visionNoiseXY = 1.8, visionNoiseHeading = 0.8;


    PoseEkf ekf;
    public Timer timer = new Timer();

    public double now = 0.0, last = 0.0, dt = 0.0;

    /*

    public Robot(HardwareMap hw, MultipleTelemetry telemetry, Alliance alliance, Pose startPose) {
        List<LynxModule> allHubs = hw.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        this.op = AUTONOMOUS;
        this.hw = hw;
        this.telemetry = telemetry;
        Robot.alliance = alliance;
        p = startPose.copy();


        follower = Constants.createFollower(hw);
        follower.setStartingPose(startPose);
        follower.update();


        timer.reset();


        ekf = new PoseEkf(
                p.getX(), p.getY(),
                processNoiseXY, processNoiseHeading,
                visionNoiseXY, visionNoiseHeading
        );

        launcher = new Launcher(hw, telemetry);
        turret = new Turret(hw, telemetry);
        hood = new Hood(hw, telemetry);
        driveTrain = new DriveTrain(hw, telemetry);
        intake = new Intake(hw, telemetry);
        led = new MyLED(hw, telemetry);
        //limelight = new Limelight(hw, telemetry);
        //limelight.update();

        //aInitLoop = false;
        // telemetry.addData("Start Pose", p);
        init();
        turret.spin.numRotations = 0;
        turret.spin.partial_rotations = 0;
        turret.spin.full_rotations = 0;
        Logger.first = true;

        k = new KinematicsCalculator(getDistanceFromGoal());

    } */

    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance, Pose startPose) {
        List<LynxModule> allHubs = hw.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        this.op = AUTONOMOUS;
        this.hw = hw;
        this.telemetry = telemetry;
        Robot.alliance = alliance;
        p = startPose.copy();


        follower = Constants.createFollower(hw);

        //follower.setStartingPose(startPose);
        follower.update();


        timer.reset();


        ekf = new PoseEkf(
                p.getX(), p.getY(),
                processNoiseXY, processNoiseHeading,
                visionNoiseXY, visionNoiseHeading
        );

        launcher = new Launcher(hw, telemetry);
        turret = new Turret(hw, telemetry);
        hood = new Hood(hw, telemetry);
        intake = new Intake(hw, telemetry);
        led = new LED(hw, telemetry);

        autoDrive = new AutoDriving(follower, telemetry);
        //limelight = new Limelight(hw, telemetry);
        //limelight.update();

        //aInitLoop = false;
        // telemetry.addData("Start Pose", p);
        init();
        turret.spin.numRotations = 0;
        turret.spin.partial_rotations = 0;
        turret.spin.full_rotations = 0;
        Logger.first = true;

        k = new KinematicsCalculator(getDistanceFromGoal());

    }

    //Teleop Controls here
    public void dualControls(GamepadEx g1, GamepadEx g2) {
        //Buttons
        g1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(() -> {
            showTelemetry = !showTelemetry;
        }));

        /*(g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).or(g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))).whenActive(new InstantCommand(() -> {
            //launcher.setLauncherState(Launcher.LauncherState.SHOOT);
            //if (launcher.controller.done) {
                if (validLaunch) {
                    intake.setUptakeState(Intake.UptakeState.ON);
                    intake.setIntakeState(Intake.IntakeState.INTAKE);
                    intakeOff = false;
                    uptakeOff = false;
                }
                else {
                    intake.setUptakeState(Intake.UptakeState.OFF);
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    intakeOff = true;
                    uptakeOff = true;
                }




            //    led.setState(MyLED.State.GREEN);
            //}
            /*else {
                intake.setUptakeState(Intake.UptakeState.OFF);
                intake.setIntakeState(Intake.IntakeState.OFF);
            //    led.setState(MyLED.State.YELLOW);
            //}


            launcherOff = false;


        })); */
        (g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))).whenInactive(new InstantCommand(() -> {
            launcherOff = true;
            intakeOff = true;
            uptakeOff = true;
            rBumper = false;
        }));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            resetPose();
        }));

        /*
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).negate().whenActive(new InstantCommand(() -> {
            launcher.setLauncherState(Launcher.LauncherState.RAMPUP);
            //intake.setIntakeState(Intake.IntakeState.INTAKE);
        })); */
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(new InstantCommand(() -> {
            launcher.setLauncherState(Launcher.LauncherState.IN);
        }));

        g2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            hood.setState(Hood.HoodState.DOWN);
        } ));
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            hood.setState(Hood.HoodState.MID);
        } ));
        g2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            hood.setState(Hood.HoodState.MIDUP);
        } ));
        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            hood.setState(Hood.HoodState.UP);
        } ));

        /*
        lTG1.whenActive(new InstantCommand(() -> {
            intake.setIntakeState(Intake.IntakeState.INTAKE);
        }));
        rTG2.whenActive(new InstantCommand(() -> {
            intake.setIntakeState(Intake.IntakeState.OUTTAKE);
        }));
        rTG2.and(lTG1).whenInactive(new InstantCommand(() -> {
            intake.setIntakeState(Intake.IntakeState.STOP);
        })); */

        //robotCentric = true;
        //g1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(this::resetPose));
        g2.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(this::flipAlliance));

        /*g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            Aim.fudgeFactor += 2.5;
        }));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            Aim.fudgeFactor -= 2.5;
        })); */
        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            turret.spin.full_rotations--;
        }));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            turret.spin.full_rotations++;
        }));
        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            //Aim.fudgeFactor = 0;
        }));
        g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            hood.increase();
        }));
        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            hood.decrease();
        }));
        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            autoDrive.toGate();
        }));
        g1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            autoDrive.toShoot();
        }));
        g1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(new InstantCommand(() -> {
            autoDrive.off();
        }));





        //.whenActive();


    }






    public void init() {
        hood.init();
        intake.init();
        launcher.init();
        //turret.init();
    }

    public void aPeriodic() {
        //telemetry.addData("path", follower.getCurrentPath());
        updateGoalCoords();
        updateRobotCoords();
        updateShooting();
        follower.update();
        //autoEndPose = follower.getPose().copy();
        if (alliance == Alliance.RED)
            autoEndPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        else
            autoEndPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

        if (logData) log();
        if (showTelemetry)
            telemetry.update();
    }

    public void aInitLoop(GamepadEx g1) {
        telemetry.addData("Alliance", alliance);
        telemetry.update();
        follower.update();
        g1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(() -> {
            alliance = Alliance.BLUE;
        }));
    }

    public void tPeriodic() {
        updateGoalCoords();
        updateRobotCoords();
        follower.update();
        autoDrive.update();
        if (showTelemetry)
            telemetry.update();
        if (logData) log();

        //turret.periodic();
        // launcher.periodic();
        //intake.periodic();
        //  hood.periodic();
        //    led.periodic();


    }

    public void tStart() {
        follower.startTeleopDrive(true);
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }

    public double getSpeed() {
        return speed;
    }
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public HardwareMap getHw() {
        return hw;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Follower getFollower() {
        return follower;
    }

    public void flipAlliance() {
        if (alliance == Alliance.BLUE)
            setAlliance(Alliance.RED);
        else
            setAlliance(Alliance.BLUE);
        //follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() + Math.toRadians(180)));
    }

    public void resetPose() {
        follower.setPose(new Pose(0, 0, Math.toRadians(90)));
    }

    public void log() {
        turret.log();
        launcher.log();
        Logger.logData(LogType.ROBOT_X, String.valueOf(getFollower().getPose().getX()));
        Logger.logData(LogType.ROBOT_Y, String.valueOf(getFollower().getPose().getY()));
        Logger.logData(LogType.ROBOT_HEADING, String.valueOf(Math.toDegrees(getFollower().getPose().getHeading())));
    }

    public double getForwardVel() {
        // Get field-centric velocity from Pedro Pathing
        Vector fieldVel = follower.getVelocity();
        double vx_field = fieldVel.getXComponent();
        double vy_field = fieldVel.getYComponent();

        // Robot heading in radians
        double theta = follower.getHeading();

        // Rotate field velocity into robot frame: forward is along robot X axis
        double v_forward = Math.cos(theta) * vx_field + Math.sin(theta) * vy_field;
        return v_forward;
    }

    // Returns robot-centric lateral velocity (meters/second)
// Positive = rightward strafe (adjust sign if your convention differs)
    public double getLateralVel() {
        Vector fieldVel = follower.getVelocity();
        double vx_field = fieldVel.getXComponent();
        double vy_field = fieldVel.getYComponent();

        double theta = follower.getHeading();

        // Rotate field velocity into robot frame: lateral is along robot Y axis
        return -Math.sin(theta) * vx_field + Math.cos(theta) * vy_field;
    }

    // Returns angular velocity (radians/second)
    public double getAngularVel() {
        return follower.getAngularVelocity(); // radians/sec
    }


    public void updateEkf() {
        double v_forward = getForwardVel();
        double v_lateral = getLateralVel();
        double omega = getAngularVel();

        last = now;
        now = timer.getElapsedTimeSeconds();
        dt = now - last;

        // Raw odometry from follower
        double odomX = follower.getPose().getX();
        double odomY = follower.getPose().getY();
        double odomTheta = follower.getHeading();

        ekf.predict(v_forward, v_lateral, omega, dt, now, odomX, odomY, odomTheta);
        updateLimelight();
    }

    public void updateShooting() {
        flightTime = k.getFlightTime();
        double d;
        //if (launcher.teleop) {
        //KinematicsCalculator.y_target_in = KinematicsCalculator.targetTele;
        d = getDistanceFromGoal();

        if (!manualR) {
            if (d > 120) r = .8;
            else r = .64;
        }

        k.setDistance(d * r);
        if (!manualRPM)
            Launcher.tele_target = k.getRPM();
        /*
        else
            Launcher.tele_target = Launcher.target_velocity; */
        Launcher.auto_target = k.getRPM();
        double hoodPos = k.getHood(launcher.current_velocity);
        if (!Launcher.teleop) {
            if (EighteenBall.firstCouple) {
                if (alliance == Alliance.RED)
                    hoodPos -= 0;
                else {
                    hoodPos -= 0;
                }
            }
            else
            if (alliance == Alliance.RED)
                hoodPos += 0;
            else
                hoodPos += 0;
        }
        if (hoodPos > 0) {
            validLaunch = true;
            if (!shotStarted || hoodAdjustment) {
                if (d < 100)
                    hood.setTarget(hoodPos);
                else {
                    hood.setTarget(hoodPos);
                    //hood.setTarget(Hood.hoodUp);
                }
            }
        } else {
            validLaunch = false;
        }
        // }
        /*
        else {
            Launcher.auto_target = 4400;
            hood.setTarget(.9);
        } */






        //Old shooting code
        /*
        if (getDistanceFromGoal() > farLaunchDist) {
            Launcher.tele_target = 5200;
            Hood.hoodIncreaseAmt = 0;
            hood.setTarget(Hood.hoodUp);
        }
        else if (getDistanceFromGoal() > goalDist) {
            Launcher.tele_target = 4200;
            Hood.hoodIncreaseAmt = 0.01;
            hood.setTarget(Hood.hoodUp);
        }
        else {
            Launcher.tele_target = 3000;
            Hood.hoodIncreaseAmt = 0;
            hood.setTarget(Hood.hoodDown);
        }
        */

    }

    public void updateLimelight() {
        if (limelight.getResult().isValid()) {
            Pose3D botPose = limelight.botPose();

            // Convert Limelight Pose3D to field x/y
            double visionX = botPose.getPosition().y; // field x
            double visionY = botPose.getPosition().x; // field y
            double visionTimestamp = now - limelight.getLatency(); // get timestamp of when limelight was last read

            // Update EKF only with x/y
            ekf.updateWithVision(visionX, visionY, visionTimestamp);
        }
    }

    public void updateRobotCoords() {
        robotX = follower.getPose().getX();
        robotY = follower.getPose().getY();
        turretX = robotX - turretOffset * Math.cos(follower.getPose().getHeading());
        turretY = robotY - turretOffset * Math.sin(follower.getPose().getHeading());
    }

    public double getDistanceFromGoal() {
        goalY = centerY;
        double vx = KinematicsCalculator.inchesToMeters(follower.getVelocity().getXComponent());
        double vy = KinematicsCalculator.inchesToMeters(follower.getVelocity().getYComponent());
        double dx = goalX - turretX - vx * flightTime;
        double dy = goalY - turretY - vy * flightTime;

        return Math.sqrt(dx * dx + dy * dy);
    }

    public void updateGoalCoords() {
        if (getDistanceFromGoal() < 100) {
            redX = 67;
            blueX = -67;
            goalY = 67;
        }
        else {
            redX = 72;
            blueX = -72;
            goalY = 72;
        }

        if (alliance == Alliance.RED) {
            goalX = redX;
        }
        else {
            goalX = blueX;
        }

        turret.updateAiming(goalX, goalY, new Pose(turretX, turretY, follower.getHeading()));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    public void outtake1() {
        if (!outtake) {
            timer.reset();
            outtake = true;
            intakeOff = false;
            intake.setIntakeState(Intake.IntakeState.SLOWOUTTAKE);
        }
        else {
            if (timer.getElapsedTimeSeconds() > .1) {
                intake.setIntakeState(Intake.IntakeState.OFF);
                intakeOff = true;
            }
        }
    }

    public boolean isInLaunchZone() {
        double theta = follower.getHeading() - Math.PI / 2;
        double h = robot_length / 2.0;
        double forwardX = Math.cos(theta);
        double forwardY = Math.sin(theta);
        double lateralX = -Math.sin(theta);
        double lateralY = Math.cos(theta);

        double flX = robotX + h * forwardX + h * lateralX;
        double flY = robotY + h * forwardY + h * lateralY;
        Pose fl = new Pose(flX, flY);

        double frX = robotX + h * forwardX - h * lateralX;
        double frY = robotY + h * forwardY - h * lateralY;
        Pose fr = new Pose(frX, frY);

        double blX = robotX - h * forwardX + h * lateralX;
        double blY = robotY - h * forwardY + h * lateralY;
        Pose bl = new Pose(blX, blY);

        double brX = robotX - h * forwardX - h * lateralX;
        double brY = robotY - h * forwardY - h * lateralY;
        Pose br = new Pose(brX, brY);

        if (inCloseZone(fl) || inCloseZone(fr) || inCloseZone(bl) || inCloseZone(br)) {
            zone = Zone.CLOSE;
            return true;
        }
        else if (inFarZone(fl) || inFarZone(fr) || inFarZone(bl) || inFarZone(br)) {
            zone = Zone.FAR;
            return true;
        }
        else {
            zone = Zone.NONE;
            return false;
        }


    }



    public boolean inCloseZone(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();
        return y > x && y > -x && y > 0;
    }

    public boolean inFarZone(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();
        return y < x - 48 && y < -x - 48 && y < -48;
    }

    public boolean intakeDone() {
        return  (intake.uptake.getCurrent(CurrentUnit.AMPS) > 4 || uptakeOff) && intake.intake.getCurrent(CurrentUnit.AMPS) > 2.4;
    }


}