/**
 * This is a subsystem file for the Limelight camera. This gives telemetry functionality to
 * the limelight, as well as additional access features, such as pipeline control.
 *
 * @author Akash Pai - 506 Pandara
 * @author Alexander Wojtulewski - 506 Pandara
 */

package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D; // ? needed ?
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight extends SubsystemBase {
    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    private Limelight3A limelight;

    //stores result of limelight
    private LLResult result;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "ll");

        //default pipeline
        //8 = red goal, 7 = obeselisque, 6 = blue goal
        setPipeline(8);

        limelight.start();
    }

    /**
     * Starts the Limelight if it is stopped
     */
    public void startLimelight() {
        limelight.start();
    }

    /**
     * Stops limelight camera
     */
    public void stopLimelight() {
        limelight.stop();
    }

    /**
     * Pipelines define a current mode for the limelight; example: AprilTags / Color Tracking - coded
     * externally on the Web interface.
     * This function changes the pipeline mode of the limelight camera
     *
     * @param pipelineIndex - Index of Limelight pipeline
     */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    @Override
    public void periodic() {
        /*
        updateTelemetry();
        update(); */
    }

    public double getLatency() {
        return result.getCaptureLatency() + result.getTargetingLatency();
    }

    /**
     * Updates all telemetry values of the limelight for testing and fetches limelight
     * result and status.
     */
    public void updateTelemetry() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose_MT2();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            telemetry.addData("Botpose", botpose.toString());
        }

        //telemetry.update();
    }

    /**
     * Updates the limelight's state to receive the most recent result. This update is
     * based on the IMU heading, which makes 3d localization more accurate.
     */
    public void update() {
        result = limelight.getLatestResult();
    }

    /**
     * @return current limelight data (May need to be manually updated)
     */
    public LLResult getResult() {
        return result;
    }

    /**
     * @return Bot pose based on the limelight's algorithm using the internal 3d map
     */
    public Pose3D botPose() {
        return result.getBotpose_MT2();
    }

}