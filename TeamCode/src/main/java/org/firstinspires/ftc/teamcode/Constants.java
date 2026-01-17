package org.firstinspires.ftc.teamcode;

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
    public static final double leftFlapUp = 0.0;
    public static final double leftFlapDown = 0.35;
    public static final double rightFlapUp = 0.75;
    public static final double rightFlapDown = 0.35;
    public static final double liftExtend = 0.76;
    public static final double liftRetract = 0.1;
    public static final double flipperRight = 0.1;
    public static final double flipperLeft = 1.0;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.832)  // Kg
            .forwardZeroPowerAcceleration(-40.13388889162883)
            .lateralZeroPowerAcceleration(-98.04551413680889)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0, 0.038))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.05, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0.0,0.00001,0.6,0.01));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(59.811501330278055)
            .yVelocity(44.07913712629183);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-100)
            .strafePodX(110)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
