package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

    public DcMotorEx launcher1; //launcher 1 = flywheel

    public DcMotorEx launcher2; //launcher 2 = counter roller
    //private VoltageSensor sensor; //measures current battery voltage

    public PDFLController controller;
    public static double counterRollerPower = .91;
    public static boolean manualCounterRoller = false;

    //pdfl values tuned in FTC Dashboard
    public static double p = 0.01;
    public static double d = 0;
    public static double f = 0.08;
    public static double l = 0;
    public static double i = 0.00001;

    public static double p2 = 0.007;
    public static double d2 = 0.01;
    public static double f2 = 0.13;
    public static double l2 = 0;
    public static double i2 = 0;


    public static double target_velocity = 0;
    public static double target_velocity_2 = 4200;
    public static double tele_target = 4500;
    public static double auto_target = 4000;
    public static boolean powerMode = false;
    public static boolean boomBoom = false;
    public static boolean pid1 = true;
    public double current_velocity = 0;
    public double current_velocity_2 = 0;
    public double prev_velocity = 0;
    public double currentPower = 0;
    public double pdfl = 0;

    private double power = 0;

    public static double test1 = 0;
    public static double test2 = 0;

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
    public double measuredV = 0;

    private boolean inBoost = false;
    private boolean inAggressive = false;
    public static boolean teleop = false;
    public boolean validLaunch = false;
    public static double threshold = 50.0;

    public enum LauncherState {
        IN,
        OUT,
        STOP,
        SHOOT
    }

    public LauncherState current = LauncherState.STOP;
    public HardwareMap hw;



    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        //init telemetry
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.hw = hardwareMap;

        //init servos based on their name in the robot's config file
        launcher1 = hardwareMap.get(DcMotorEx.class, "cm1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "cm0");

        //sensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double d = Math.abs(target_velocity + current_velocity);
        validLaunch = d < threshold;
        measuredV = hw.voltageSensor.iterator().next().getVoltage();
        current = LauncherState.OUT;
        updateShooter();
        telemetry.addData("Target Velocity 1", target_velocity);
        telemetry.addData("Target Velocity 2", target_velocity_2);
        telemetry.addData("Current Velocity 1", -current_velocity);
        telemetry.addData("Current Velocity 2", -current_velocity_2);
        telemetry.addData("Volts", measuredV);
        telemetry.addData("Valid", validLaunch);
        telemetry.update();
        log();

    }

    public void setLauncherState(LauncherState state) {
        current = state;
    }


    public void periodic() {
        double d = Math.abs(target_velocity + current_velocity);
        validLaunch = d < threshold;
        measuredV = hw.voltageSensor.iterator().next().getVoltage();
        //if (Robot.logData) log();
        if (current != LauncherState.STOP) {
            if (teleop)
                target_velocity = tele_target;
            else target_velocity = auto_target;
        }

        else  {
            target_velocity = 0;
            launcher1.setPower(0);
            launcher2.setPower(0);
        }
        updateShooter();


        if (showTelemetry) {
            telemetry.addData("Target Velocity 1", target_velocity);
            telemetry.addData("Target Velocity 2", target_velocity_2);
            telemetry.addData("Current Velocity 1", current_velocity);
            telemetry.addData("Current Velocity 2", current_velocity_2);
            telemetry.addData("Volts", measuredV);
            telemetry.addData("Valid", validLaunch);

            //telemetry.addData("Done", controller.done);
            //telemetry.addData("Num Done", numDone);
            telemetry.addData("Launcher 1 Current", launcher1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Launcher 2 Current", launcher2.getCurrent(CurrentUnit.AMPS));
        }


    }

    public void periodicShootingTest(Robot r) {
        double d = Math.abs(target_velocity - current_velocity);
        validLaunch = d < threshold;
        updateShooter();
        if (true) {
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
        telemetry.addData("Current Velocity", -current_velocity);
        telemetry.addData("State", current);
        telemetry.addData("Done", controller.done);
        telemetry.addData("Volts", measuredV);
        telemetry.addData("Valid", validLaunch);
        telemetry.update();
        log();
        r.intake.log();
    }

    public void updateShooter() {
        double pdfl = 0;
        prev_velocity = current_velocity;
        current_velocity = tickstoRPM(launcher2.getVelocity());
        current_velocity_2 = tickstoRPM(launcher1.getVelocity());
        controller.update(-current_velocity, target_velocity);
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
                launcher2.setPower(counterRollerPower * (12.01 / measuredV));
            }
            else {
                if (!powerMode) {
                    //target_velocity *= 13.0 / measuredV;
                    if (target_velocity > -current_velocity) {
                        launcher1.setPower(1);
                    } else {
                        launcher1.setPower(0);
                    }


                    /*if (target_velocity_2 > -current_velocity_2) {
                        launcher2.setPower(1);
                    } else {
                        launcher2.setPower(0);
                    } */
                    if (current == LauncherState.SHOOT)
                        launcher2.setPower(counterRollerPower * (12.01 / measuredV));
                    else
                        launcher2.setPower(counterRollerPower * (12.01 / measuredV));
                }
                else {
                    launcher1.setPower(test1);
                    launcher2.setPower(test2);
                }
            }
        }
    }

    public void setTarget(double target) {
        target_velocity = target;
    }

    public boolean getValidLaunch() {
        return validLaunch;
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