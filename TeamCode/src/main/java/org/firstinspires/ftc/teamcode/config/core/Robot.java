package org.firstinspires.ftc.teamcode.config.core;

import static org.firstinspires.ftc.teamcode.config.core.util.Opmode.*;

import android.graphics.Interpolator;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.commands.*;
import org.firstinspires.ftc.teamcode.config.core.paths.AutoDriving;
import org.firstinspires.ftc.teamcode.config.core.util.*;
import org.firstinspires.ftc.teamcode.config.util.KinematicsCalculator;
import org.firstinspires.ftc.teamcode.config.util.PoseEkf;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.*;
import org.firstinspires.ftc.teamcode.config.util.Timer;

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
    public static boolean showTelemetry = false;


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
    public MyLED led;

    public KinematicsCalculator k;
    public boolean slowMode;
    public Limelight limelight;
    public Intake intake;
    public boolean robotCentric = false;
    public static Alliance alliance = Alliance.RED;



    public static double centerX = 67, centerY = 67;
    public static double centerX2 = centerX, centerY2 = centerY;
    public static double goalY = centerY;
    public static double redX = centerX;
    public static double blueX = -centerX;

    public static boolean manualR = false;
    public static boolean manualRPM = false;
    public boolean shotStarted = false;
    public static Pose shootPose;




    //double centerX = redX, centerY = goalY;
    double rightWallX = centerX - 7, rightWallY = centerY;  // right wall center
    double frontWallX = centerX, frontWallY = centerY - 7;

    double centerXBlue = 72,
     rightWallXBlue = 65,
     frontWallXBlue = 72;

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
    public double turretX, turretY;
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
        led = new MyLED(hw, telemetry);

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

        //g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new Fire3(this));

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
            Aim.fudgeFactor = 0;
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
        follower.startTeleopDrive();
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

        double x, y, heading;
        Pose f = follower.getPose();
        if (f.getHeading() > Math.toRadians(45)) {
            if (f.getHeading() > Math.toRadians(135))
                heading = Math.toRadians(180);
            else
                heading = Math.toRadians(90);
        }
        else {
            if (f.getHeading() < Math.toRadians(-45)) {
                if (f.getHeading() < -135)
                    heading = Math.toRadians(-180);
                else
                    heading = Math.toRadians(-90);
            }
            else
                heading = 0;
        }
        if (f.getX() < 0) {
            x = cornerBlueBack.getX();
            y = cornerBlueBack.getY();
        }

        else {
            x = cornerRedBack.getX();
            y = cornerRedBack.getY();
        }
        //follower.setPose(new Pose(x, y, heading));
        follower.setPose(new Pose(0, 0, follower.getHeading()));
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
        double d;
        //if (launcher.teleop) {
            KinematicsCalculator.y_target_in = KinematicsCalculator.targetTele;
            d = getDistanceFromGoal();

            if (!manualR) {
                if (d > 120) r = .9;
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
            if (hoodPos > 0) {
                validLaunch = true;
                if (!shotStarted) {
                    if (getDistanceFromGoal() < 100)
                        hood.setTarget(hoodPos);
                    else {
                        hood.setTarget(Hood.hoodUp);
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
        turretX = follower.getPose().getX() - turretOffset * Math.cos(follower.getPose().getHeading());
        turretY = follower.getPose().getY() - turretOffset * Math.sin(follower.getPose().getHeading());
    }

    public double getDistanceFromGoal() {
        rightWallX = centerX - 7;
        rightWallY = centerY;  // right wall center
        frontWallX = centerX;
        frontWallY = centerY - 7;
        goalY = centerY;
        double vx = 0;//follower.getVelocity().getXComponent();
        double vy = 0;//follower.getVelocity().getYComponent();
        if (alliance == Alliance.RED) {
            double goalX = centerX;
            double dx = goalX - turretX - vx * flightTime;
            double dy = goalY - turretY - vy * flightTime;
            return Math.sqrt(dx * dx + dy * dy);
        }
        else  {
            double goalX = blueX;
            double dx = goalX - turretX - vx * flightTime;
            double dy = goalY - turretY - vx * flightTime;
            return Math.sqrt(dx * dx + dy * dy);
        }
    }

    public void updateGoalCoords() {
        double t;  // blend factor
        double robotY = follower.getPose().getY();
        double robotX = follower.getPose().getX();
        //if (alliance == Alliance.RED) {
            if (getDistanceFromGoal() < 100) {
                if (robotY > robotX) {
                    // --- LEFT SIDE OF DIAGONAL → use distance from FRONT edge (y=72)
                    double dist = Math.abs(72 - robotY);  // 0 at edge, maxDist at diagonal
                    t = 1.0 - (dist / maxDist);
                    t = Math.min(1.0, Math.max(0.0, t));

                    redX = lerp(centerX, centerX2, t);
                    goalY = lerp(centerY, centerY2, t);

                } else {
                    // --- RIGHT SIDE OF DIAGONAL → use distance from RIGHT edge (x=72)
                    double dist = Math.abs(72 - robotX);  // same logic
                    t = 1.0 - (dist / maxDist);
                    t = Math.min(1.0, Math.max(0.0, t));

                    redX = lerp(centerX, centerX2, t);
                    goalY = lerp(centerY, centerY2, t);
                }
            }
            else {
                redX = 72;
                blueX = 72;
                goalY = 72;
            }
        /*}
        else {
            if (robotY > robotX) {
                // --- LEFT SIDE OF DIAGONAL → use distance from FRONT edge (y=72)
                double dist = Math.abs(72 - robotY);  // 0 at edge, maxDist at diagonal
                t = 1.0 - (dist / maxDist);
                t = Math.min(1.0, Math.max(0.0, t));

                redX = lerp(centerX, frontWallX, t);
                goalY = lerp(centerY, frontWallY, t);

            } else {
                // --- RIGHT SIDE OF DIAGONAL → use distance from RIGHT edge (x=72)
                double dist = Math.abs(-72 - robotX);  // same logic
                t = 1.0 - (dist / maxDist);
                t = Math.min(1.0, Math.max(0.0, t));

                blueX = lerp(centerX, rightWallX, t);
                goalY = lerp(-centerY, -rightWallY, t);
            }
        } */
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

    public boolean intakeDone() {
        return  (intake.uptake.getCurrent(CurrentUnit.AMPS) > 3 || uptakeOff) && intake.intake.getCurrent(CurrentUnit.AMPS) > 1.5;
    }


}