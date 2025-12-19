package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Pinpoint;
import org.firstinspires.ftc.teamcode.Shooter;

import java.nio.file.Paths;
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroPathingVisualizer extends LinearOpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)


    Chassis ch;
    Shooter shooterLeft;
    Shooter shooterRight;
    Servo launchFlapLeft;
    Servo launchFlapRight;
    Shooter collectorBack;
    Shooter collectorFront;
    Servo flipper;
    Pinpoint pinpoint;
    GoalTagLimelight limelight;
    private int startDelay = 0;
    private int teamID;
    private boolean testingMode = false;
    private boolean shooting = false;
    private double collectorPower = 0.53;
    private double maxPower = 0.5;

    private final Pose startPose = new Pose(61.850, 8.348, Math.toRadians(90));
    private final Pose launchPose = new Pose(60.592, 19.324);
    private final Pose readyPickUp1 = new Pose(40.222, 35.289);
    private final Pose collectLine1 = new Pose(15.368, 35.478);
    private final Pose readyPickUp2 = new Pose(40.032, 59.763);
    private final Pose collectLine2 = new Pose(15.368, 59.763);

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;


    @Override
    public void runOpMode() {
        setPathState(0);


        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        ch = new Chassis(hardwareMap);
        
        pinpoint = new Pinpoint(hardwareMap,ch,telemetry,false);

        collectorFront = new Shooter(hardwareMap,"collectorFront", false);

        collectorBack = new Shooter(hardwareMap,"collectorBack", false);

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);

        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap,telemetry);

        pinpoint.odo.resetPosAndIMU();
        pinpoint.odo.recalibrateIMU();

        Pose2D pose = pinpoint.odo.getPosition();

        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);

        do {
            pinpoint.odo.update();
            limelight.readObelisk(telemetry);
            GlobalStorage.setPattern(limelight.getObelisk());

            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", teamID);
            telemetry.addData("Testing Mode", testingMode);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("collectorPower", collectorPower);

            if (gamepad1.bWasPressed()) {
                //goalTag.targetAprilTagID = 24;
                teamID = 24;
                GlobalStorage.setAlliance(24);
            } else if (gamepad1.xWasPressed()) {
                //goalTag.targetAprilTagID = 20;
                teamID = 20;
                GlobalStorage.setAlliance(20);
            } else if (gamepad1.yWasPressed()) {
                startDelay += 2;
            } else if (gamepad1.aWasPressed()) {
                startDelay -= 1;
            } else if (gamepad1.leftStickButtonWasPressed()) {
                testingMode = true;
            }

            telemetry.addData("Heading Scalar", pinpoint.odo.getYawScalar());
            telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Initial Heading (deg) MAKE SURE ITS 0", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
            telemetry.update();
        } while (opModeInInit());

        while (opModeIsActive()) {
            sleep(1000*startDelay);

            limelight.setTeam(teamID);

            collectorBack.setPower(collectorPower);
            collectorFront.setPower(collectorPower);

            launchFlapLeft.setPosition(0.3);
            launchFlapRight.setPosition(0.4);

            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
            break;
        }
    }

    public void buildPaths() {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPose, launchPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(launchPose, readyPickUp1)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(readyPickUp1, collectLine1)
                    )
                    .addParametricCallback(30, collectLeft)
                    .addParametricCallback(60, collectRight)
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(collectLine1, launchPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPose, readyPickUp2)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(readyPickUp2, collectLine2)
                    )
                    .addParametricCallback(30, collectLeft)
                    .addParametricCallback(60, collectRight)
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(collectLine2, launchPose)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1, maxPower, false);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    fireVolleySorted();
                    follower.followPath(Path2, maxPower, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path3, maxPower, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path4, maxPower, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path5, maxPower, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path6, maxPower,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path7, maxPower, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;

        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void fireVolleySorted() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double velRight = 0;
        double velLeft = 0;
        while (timer.seconds() < 0.25) {
            limelight.process(telemetry);
            velLeft = (limelight.getRange()+100.99)/7.3712;
            velRight = (limelight.getRange()+100.99)/7.3712;
        }
        if (limelight.getObelisk().equals("PGP") && !testingMode) {
            fireShooterLeft(velLeft);
            fireShooterRight(velRight);
            flipper.setPosition(1);
            sleep(1000);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("GPP") && !testingMode) {
            fireShooterRight(velRight);
            fireShooterLeft(velLeft);
            sleep(1000);
            flipper.setPosition(1);
            sleep(1000);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("PPG") && !testingMode) {
            fireShooterLeft(velLeft);
            sleep(1000);
            flipper.setPosition(1);
            sleep(1000);
            fireShooterLeft(velLeft);
            fireShooterRight(velRight);
        }
    }
    public void fireShooterLeft(double velocity) {
        shooting = true;
        shooterLeft.targetVelocity = velocity;
        ElapsedTime timer = new ElapsedTime();

        while (!shooterLeft.atSpeed()) {
            shooterLeft.overridePower();
        }
        timer.reset();
        launchFlapLeft.setPosition(0);
        while (timer.seconds() < 0.5) {
            shooterLeft.overridePower();
        }
        launchFlapLeft.setPosition(0.3);
        while (timer.seconds() < 1) {
            shooterLeft.overridePower();
        }
    }
    public void fireShooterRight(double velocity) {
        shooting = true;
        shooterRight.targetVelocity = velocity;
        ElapsedTime timer = new ElapsedTime();

        while (!shooterRight.atSpeed()) {
            shooterRight.overridePower();
        }
        timer.reset();
        launchFlapRight.setPosition(0.7);
        while (timer.seconds() < 0.5) {
            shooterRight.overridePower();
        }
        launchFlapRight.setPosition(0.4);
        while (timer.seconds() < 1) {
            shooterRight.overridePower();
        }
    }

    Runnable collectLeft = () -> {
        flipper.setPosition(1);
        sleep(500);
        flipper.setPosition(0.525);
    };
    Runnable collectRight = () -> {
        flipper.setPosition(0.1);
        sleep(500);
        flipper.setPosition(0.525);
    };
}
