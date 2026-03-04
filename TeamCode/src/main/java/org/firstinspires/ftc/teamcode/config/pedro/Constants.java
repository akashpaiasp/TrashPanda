package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PinpointLocalizer localizer;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.4)
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
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0.0001, 0.000005, 0.6, 0.01))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(.3, .06756, .00203));

    public static MecanumConstants driveConstants = new MecanumConstants()
            //Motor names in the robot's config file
            .leftFrontMotorName("em2")
            .leftRearMotorName("em3")
            .rightFrontMotorName("cm2")
            .rightRearMotorName("cm3")

            //Motor directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(75)
            .yVelocity(55);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.713)
            .strafePodX(-.73) //47 //145.5
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("ci1")
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(.975, 50, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        localizer = new PinpointLocalizer(hardwareMap, localizerConstants);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .setLocalizer(localizer)
                .build();
    }
}
