package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;

import java.util.ArrayList;

public class AxonContinuous {
    private CRServo c;
    private AnalogInput a;

    public  double numRotations = 0;
    private boolean forward = true;
    public double lastVoltage = 0;
    private static double threshold = 1;
    public  double partial_rotations = 0;
    public  double full_rotations = 0;
    private double servoPower = 0;
    public Timer resetTime = new Timer();
    public double lastTimeNeg = 0;
    public double lastTimePos = 0;
    public static double timeThreshold = 0.05;
    public ArrayList<Double> volts = new ArrayList<>();
    public boolean first = true;
    public double volt;



    public AxonContinuous(HardwareMap hwMap, String hw1, String hw2) {
        c = hwMap.get(CRServo.class, hw1);
        a = hwMap.get(AnalogInput.class, hw2);
        resetTime.reset();
        lastVoltage = getVolts();
    }

    public CRServo getC() {
        return c;
    }

    public AnalogInput getA() {
        return a;
    }

    public double getVolts() {
        //if (a.getVoltage() < 3.2 && a.getVoltage() > 0.1)
        volt = Math.round(a.getVoltage() * 100.0) / 100.0;
        /*
        if (first) {
            for (double d : volts)
                volts.add(volt);
            first = false;
        }
        else {
            volts.add(volt);
            volts.remove(0);
        }
        double sum = 0;
        for (Double d : volts)
                sum += d;
        return sum / volts.size(); */
        return volt;


    }

    public void update() {

    }

    public void setPower(double power) {
        servoPower = power;//0.5 + power / 2.0;
        c.setPower(servoPower);
    }

    //make it so that u cant do multiple thingys within a certain time period (time threshold)

    public void calculate() {
        double v = getVolts();
        if (Math.abs(v - lastVoltage) > threshold) {
            if (v > lastVoltage && resetTime.getElapsedTimeSeconds() - lastTimeNeg > timeThreshold) {
                full_rotations--;
                lastTimeNeg = resetTime.getElapsedTimeSeconds();
            }
            else if (v < lastVoltage && resetTime.getElapsedTimeSeconds() - lastTimePos > timeThreshold) {
                full_rotations++;
                lastTimePos = resetTime.getElapsedTimeSeconds();
            }
        }

        partial_rotations = v / 3.3;
        numRotations = full_rotations + partial_rotations;
        lastVoltage = v;
    }

    public double getNumRotations() {
        return numRotations;
    }

    public double getServoPower() {
        return servoPower;
    }

    public double getFull_rotations() {
        return full_rotations;
    }

    public double getPartial_rotations() {
        return partial_rotations;
    }
}