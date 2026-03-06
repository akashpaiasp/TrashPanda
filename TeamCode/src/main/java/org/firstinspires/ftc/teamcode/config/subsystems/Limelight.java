//sample code so far for limelight 3A - stolen from samples

package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D; // ? needed ?
import org.firstinspires.ftc.robotcore.external.Telemetry;

//subsystem of the limelight 3A
public class Limelight extends SubsystemBase {
    //for telemetry of the limelight
    private MultipleTelemetry telemetry;

    //limelight class - should probably figure out whats inside
    private Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        //links to limelight - need to make sure it connect properly
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "ll");

        setPipeline(8);

        //8 = red goal, 7 = obeselisque, 6 = blue goal

        //I think this is the best place to put this function.
        limelight.start();
    }

    private LLResult result;

    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    //pipelines define a current mode for the limelight; example: AprilTags / Color Tracking - coded externally on
    // the Web interface - I need to do more research but this is the gist
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void periodic() {
        /*
        updateTelemetry();
        update(); */
    }

    public double getLatency() {
        return result.getCaptureLatency() + result.getTargetingLatency();
    }

    //bascially copied code
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
            Pose3D botpose = result.getBotpose();
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


    //update limelight based off of imu heading, makes 3d localization more accurate.
    public void update() {
        result = limelight.getLatestResult();
    }

    public LLResult getResult() {
        return result;
    }

    public Pose3D botPose() {
        return result.getBotpose();
    }

    public Pose getRobotPosFromTarget() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            double angle = botPose.getOrientation().getYaw(AngleUnit.DEGREES) + 180;
            if (angle > 360) angle -= 360;

            return new Pose(botPose.getPosition().x / .0254, botPose.getPosition().y / .0254, Math.toRadians(angle));
        }
        return new Pose();

    }


}
