package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.robotConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-120.804004394470101)
            .lateralZeroPowerAcceleration(-51.48416846267221)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0815, 0.0, 0.004, 0.025))
            .translationalPIDFSwitch(4)
            .headingPIDFCoefficients(new PIDFCoefficients(1.715, 0.0, 0.06, 0.005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.003, 0.0, 0.000165, 0.6, 0.001))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005)
            .useSecondaryDrivePIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName(robotConstants.RobotConstants.frontLeftMotorName)
            .leftRearMotorName(robotConstants.RobotConstants.backLeftMotorName)
            .rightFrontMotorName(robotConstants.RobotConstants.frontRightMotorName)
            .rightRearMotorName(robotConstants.RobotConstants.backRightMotorName)
            .leftFrontMotorDirection(DcMotor.Direction.FORWARD)
            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotor.Direction.REVERSE)
            .rightRearMotorDirection(DcMotor.Direction.REVERSE)
            .xVelocity(75.426799623985)
            .yVelocity(58.329954612912175);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.5)
            .strafePodX(4.45)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints
            (0.99, 100, 1.24, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}