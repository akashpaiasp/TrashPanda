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
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import java.util.List;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D; // ? needed ?
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight extends SubsystemBase {
    //pipeline numbers
    final static int APRILTAG = 0;
    final static int NEURALDETECTOR = 1;


    //Telemetry = text that is printed on the driver station while the robot is running
    private MultipleTelemetry telemetry;

    private Limelight3A limelight;

    //stores result of limelight
    private LLResult result;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "ll");

        //default pipeline
        //8 = red goal, 7 = obelisk, 6 = blue goal
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
        //updateTelemetry();
        update();
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

        // Access general information
        Pose3D botpose = result.getBotpose_MT2();
        double captureLatency = result.getCaptureLatency();
        double targetingLatency = result.getTargetingLatency();
        double parseLatency = result.getParseLatency();
        telemetry.addData("LL Latency", result.isValid() ? captureLatency + targetingLatency : 0.0);
        telemetry.addData("Parse Latency", result.isValid() ? parseLatency : 0.0);
        telemetry.addData("PythonOutput", result.isValid() ? java.util.Arrays.toString(result.getPythonOutput()) : "Null");

        telemetry.addData("tx", result.isValid() ? result.getTx() : 0.0);
        telemetry.addData("txnc", result.isValid() ? result.getTxNC() : 0.0);
        telemetry.addData("ty", result.isValid() ? result.getTy() : 0.0);
        telemetry.addData("tync", result.isValid() ? result.getTyNC() : 0.0);

        telemetry.addData("Botpose", result.isValid() ? botpose.toString() : "Null");

        telemetry.update();
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
        return getResult().getBotpose_MT2();
    }

    /**
     * A direct access that returns raw results from the neural detector pipeline
     * on the limelight.
     * @return list of raw detector results from the neural network pipeline
     * if pipeline is equal to NEURALDETECTOR. Else, returns null.
     */
    public List<DetectorResult> getDetectorResults() {
        if(limelight.getStatus().getPipelineIndex() == NEURALDETECTOR)
        {
            return limelight.getLatestResult().getDetectorResults();
        }
        return null;
    }

}