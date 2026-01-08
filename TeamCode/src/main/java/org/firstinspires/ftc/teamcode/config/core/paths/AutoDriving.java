package org.firstinspires.ftc.teamcode.config.core.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import static org.firstinspires.ftc.teamcode.config.core.paths.autonomous.EighteenBall.*;

//Paths for automatic driving in teleop
public class AutoDriving {
    private Follower f;
    private Telemetry telemetry;
    private boolean autoDrive;
    public AutoDriving(Follower f, Telemetry t) {
        this.f = f;
        telemetry = t;
        autoDrive = false;
    }
    //Define Poses
    Pose shoot = new Pose(15, 17, .62);
    Pose shootBlue = new Pose(-51, 17, .62 - Math.PI);
    //Run Paths
    public void toGate() {
        autoDrive = true;
        f.followPath(getGatePath());
    }

    public void toShoot() {
        autoDrive = true;
        f.followPath(getShootPath());
    }


    public PathChain getShootPath() {
        Pose p = f.getPose();

        if (Robot.alliance == Alliance.RED) {

            return f.pathBuilder()
                    .addPath(new BezierLine(p, shoot))
                    .setLinearHeadingInterpolation(p.getHeading(), shoot.getHeading())
                    .build();
        }
        else {
            return f.pathBuilder()
                    .addPath(new BezierLine(p, shootBlue))
                    .setLinearHeadingInterpolation(p.getHeading(), shootBlue.getHeading())
                    .build();
        }
    }


    public PathChain getGatePath() {
        Pose p = f.getPose();

        if (Robot.alliance == Alliance.RED) {

            return f.pathBuilder()
                    .addPath(new BezierLine(p, strafeGate))
                    .setLinearHeadingInterpolation(p.getHeading(), strafeGate.getHeading())
                    .addPath(new BezierLine(strafeGate, gate))
                    .setBrakingStart(.8)
                    .setLinearHeadingInterpolation(strafeGate.getHeading(), gate.getHeading())
                    .build();
        }
        else {
            return f.pathBuilder()
                    .addPath(new BezierLine(p, strafeGateBlue))
                    .setLinearHeadingInterpolation(p.getHeading(), strafeGateBlue.getHeading())
                    .addPath(new BezierLine(strafeGateBlue, gateBlue))
                    .setBrakingStart(.8)
                    .setLinearHeadingInterpolation(strafeGateBlue.getHeading(), gateBlue.getHeading())
                    .build();
        }
    }




    public void off() {
        autoDrive = false;
        f.startTeleopDrive();
    }

    public void update() {
        if (!f.isBusy() && autoDrive) {
            off();
        }
        telemetry();
    }

    public void telemetry() {
        telemetry.addData("Auto Drive", autoDrive ? "ON" : "OFF");
    }


}