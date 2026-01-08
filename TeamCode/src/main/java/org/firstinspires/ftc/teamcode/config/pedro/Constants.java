package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.7)
            .forwardZeroPowerAcceleration(-38)
            .lateralZeroPowerAcceleration(-81)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.07, 0.06))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013, 0, 0.001, 0.6, 0.0001))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.13, 0, 0.01, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.05, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0.0001, 0.000005, 0.6, 0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            //Motor names in the robot's config file
            .leftFrontMotorName("cm3")
            .leftRearMotorName("cm2")
            .rightFrontMotorName("em2")
            .rightRearMotorName("em3")

            //Motor directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(75)
            .yVelocity(55);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.713)
            .strafePodX(-.73 - 3.8/2) //47 //145.5
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("ci0")
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
