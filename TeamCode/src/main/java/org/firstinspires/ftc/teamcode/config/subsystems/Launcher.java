package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;
import org.firstinspires.ftc.teamcode.config.util.Timer;

/*Sample subsystem class. Subsystems are anything on the robot that is not the drive train
such as a claw or a lift.
*/

//28 ticks per rotation
@Config
public class Launcher extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    public DcMotorEx launcher1;

    public DcMotorEx launcher2;

    public PDFLController controller;

    //pdfl values tuned in FTC Dashboard
    public static double p = 0.007;
    public static double d = 0.01;
    public static double f = 0.13;
    public static double l = 0;
    public static double i = 0.00035;

    public static double p2 = 0.007;
    public static double d2 = 0.01;
    public static double f2 = 0.13;
    public static double l2 = 0;
    public static double i2 = 0;


    public static double target_velocity = 0;
    public static double tele_target = 4500;
    public static double auto_target = 4000;
    public static boolean powerMode = false;
    public static boolean boomBoom = true;
    public static boolean pid1 = true;
    public double current_velocity = 0;
    public double prev_velocity = 0;
    public double currentPower = 0;
    public double pdfl = 0;

    private double power = 0;

    public static double test1 = 0;
    public static double test2 = 0;
    public static boolean shoot = false;

    public long lastUpdateTime = 0;
    private Timer timer = new Timer();
    private double delta_time = 0;
    private long last_time = 0;
    private long curr_time = 0;
    private int last_position = 0;
    private int curr_position = 0;
    private int delta_pos = 0;
    private boolean ramped = false;
    private int numDone = 0;
    public boolean shotDetected = false;

    public static double boostTime = 0.14;
    public static double RECOVERY_THRESHOLD = 100;
    public static double DROP_THRESHOLD = .1;
    public static double FALL_THRESHOLD = .83;

    private boolean inBoost = false;
    private boolean inAggressive = false;
    public static boolean teleop = false;



    public enum LauncherState {
        IN,
        OUT,
        STOP,
    }

    public LauncherState current = LauncherState.STOP;
    public HardwareMap hw;



    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.hw = hardwareMap;

        //init servos based on their name in the robot's config file
        launcher1 = hardwareMap.get(DcMotorEx.class, "em0");
        launcher2 = hardwareMap.get(DcMotorEx.class, "em1");
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = new PDFLController(p, d, f, l, i);
        controller.reset();
        timer.reset();
    }

    /*Periodic method gets run in a loop during auto and teleop.
    The telemetry gets updated constantly so you can see the status of the subsystems */
    public void periodicTest() {
        last_position = curr_position;
        last_time = curr_time;

        //current_velocity = tickstoRPM(launcher1.getVelocity());


        curr_position = launcher1.getCurrentPosition();
        //curr_time = timer.getElapsedTime();

        //delta_time = (timer.getElapsedTime() - last_time) / 1000.0;
        //delta_pos = curr_position - last_position;
        current_velocity = tickstoRPM(launcher1.getVelocity());//tickstoRPM((double)(delta_pos) / delta_time);


        controller.update(current_velocity, target_velocity);
        pdfl = controller.run();
        power =  target_velocity != 0 ? pdfl : 0;
        //power = test1;
        power = Range.clip(power, -1, 1);
        currentPower = power;

        // Clamp power between -1 and 1
        //power = Math.max(-1, Math.min(1, power));
        if (!powerMode) {
            launcher1.setPower(power);
            launcher2.setPower(power);
        }
        else {
            launcher1.setPower(test1);
            launcher2.setPower(test2);
        }
        if (pid1)
            controller.updateConstants(p, d, f, l, i);
        else
            controller.updateConstants(p2, d2, f2, l2, i2);
        telemetry.addData("Launcher1 Velocity", current_velocity);
        telemetry.addData("Launcher2 Velocity", tickstoRPM(launcher2.getVelocity()));

        telemetry.addData("pdfl", pdfl);

        telemetry.addData("Target", target_velocity);

        telemetry.addData("p", controller.getP());
        telemetry.addData("d", controller.getD());
        telemetry.addData("f", controller.getF());
        telemetry.addData("l", controller.getL());
        telemetry.addData("i", controller.getI());

        telemetry.addData("dt", controller.getDelta_time());
        telemetry.addData("de", controller.getDelta_error());

        telemetry.addData("power", power);

        telemetry.addData("Reached target", controller.getReached());
        telemetry.addData("Rise time", controller.getRiseTime());

        telemetry.addData("Settled", controller.isSettled());
        telemetry.addData("Settling time", controller.getSettlingTime());

        telemetry.addData("Error", controller.getError());
        telemetry.addData("Reached Threshold" , controller.getReachedThreshold());

        telemetry.addData("Position", launcher1.getCurrentPosition());

        telemetry.addData("Delta Time 2", delta_time);
        telemetry.addData("Delta Position", delta_pos);

        telemetry.addData("Total Error", controller.getTot_error());

        telemetry.addData("Done", controller.done);



        telemetry.update();
        log();

    }

    public void setLauncherState(LauncherState state) {
        current = state;
    }


    public void periodic() {
        //if (Robot.logData) log();
        if (current == LauncherState.OUT) {
            if (teleop)
                target_velocity = tele_target;
            else target_velocity = auto_target;
        }
        else if (current == LauncherState.STOP) {
            target_velocity = 0;
            launcher1.setPower(0);
            launcher2.setPower(0);
        }
        updateShooter();



        telemetry.addData("Target Velocity", target_velocity);
        telemetry.addData("Current Velocity", current_velocity);
        //telemetry.addData("Done", controller.done);
        //telemetry.addData("Num Done", numDone);
        telemetry.addData("Launcher Current", launcher1.getCurrent(CurrentUnit.AMPS));


    }

    public void periodicShootingTest(Robot r) {
        updateShooter();
        if (shoot) {
            if (controller.done) {

                r.intake.setUptakeState(Intake.UptakeState.ON);
                r.intake.setIntakeState(Intake.IntakeState.INTAKE);
            }
            else {
                r.intake.setUptakeState(Intake.UptakeState.OFF);
                r.intake.setIntakeState(Intake.IntakeState.OFF);
            }

        }
        else {
            r.intake.setUptakeState(Intake.UptakeState.OFF);
            r.intake.setIntakeState(Intake.IntakeState.OFF);
        }




        r.intake.periodic();
        telemetry.addData("Target Velocity", target_velocity);
        telemetry.addData("Current Velocity", current_velocity);
        telemetry.addData("State", current);
        telemetry.addData("Done", controller.done);
        telemetry.addData("Volts", hw.voltageSensor.iterator().next().getVoltage());
        telemetry.update();
        log();
        r.intake.log();
    }

    public void updateShooter() {
        double pdfl = 0;
        prev_velocity = current_velocity;
        current_velocity = tickstoRPM(launcher1.getVelocity());
        controller.update(current_velocity, target_velocity);

        // 1) Detect shot
        //if (!inBoost/* && !inAggressive) {
        double drop = prev_velocity - current_velocity;
        if (drop > (DROP_THRESHOLD * target_velocity)) { //|| current_velocity < target_velocity * FALL_THRESHOLD) {
            shotDetected = true;
            timer.reset();
        }
        if (shotDetected && timer.getElapsedTimeSeconds() > 0.02)
            shotDetected = false;
        if (target_velocity < 3500)
            controller.updateConstants(p2, d, f, l, i);
        else
            controller.updateConstants(p, d, f, l, i);
        pdfl = controller.run();
        //}

        // 2) BOOST PHASE
    /*
        if (inBoost) {
            launcher1.setPower(1);
            launcher2.setPower(1);

            if (timer.getElapsedTimeSeconds() > boostTime) {
                inBoost = false;
                //inAggressive = true;
            }
            return; // skip PID this cycle
        }

        if (current == LauncherState.STOP)
            pdfl = 0;
        // 3) AGGRESSIVE PID RECOVERY
            /*
            if (inAggressive) {

                if (Math.abs(target_velocity - current_velocity) < RECOVERY_THRESHOLD) {
                    inAggressive = false;
                }

                controller.updateConstants(p2, d2, f2, l2, i2);
                pdfl = controller.run();
            } */
        // 4) Set Power (steady state)
        if (!(current == LauncherState.STOP)) {
            if (!boomBoom) {
                launcher1.setPower(pdfl);
                launcher2.setPower(pdfl);
            }
            else {
                if (target_velocity > current_velocity) {
                    launcher1.setPower(1);
                    launcher2.setPower(1);
                }
                else {
                    launcher1.setPower(0);
                    launcher2.setPower(0);
                }
            }
        }
    }

    public void setTarget(double target) {
        target_velocity = target;
    }

    public double tickstoRPM(double velocity) {
        return velocity * 60.0/28.0;
        //return
    }

    public void init() {
        setLauncherState(LauncherState.STOP);
        launcher1.setPower(0);
        launcher2.setPower(0);
    }

    public void log() {
        Logger.logData(LogType.LAUNCHER_TARGET, String.valueOf(target_velocity));
        Logger.logData(LogType.LAUNCHER_VELOCITY, String.valueOf(current_velocity));
        Logger.logData(LogType.LAUNCHER_SETTLED, String.valueOf(controller.done));
        Logger.logData(LogType.BOOST, String.valueOf(inBoost));
        Logger.logData(LogType.AGGRESSIVE, String.valueOf(inAggressive));
        Logger.logData(LogType.LAUNCHER_POWER, String.valueOf(launcher1.getPower()));
    }


    //.1 = 140
    //.2 = 380
    //.3 = 640
    //.4 = 960
    //.5 = 1230
    //.6 = 1500
    //.7 = 1790
    //.8 = 2010
    //.9 = 2310
    //1 = 2430

}