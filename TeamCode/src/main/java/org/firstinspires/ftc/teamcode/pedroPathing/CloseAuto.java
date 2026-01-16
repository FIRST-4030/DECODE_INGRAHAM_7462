package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Shooter;


@Autonomous(name = "CloseAuto", group = "Autonomous")
@Configurable // Panels
public class CloseAuto extends LinearOpMode {

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
    GoalTagLimelight limelight;
    Poses autoPoses = new Poses();

    private int startDelay = 0;
    private boolean testingMode = false;
    double line1Y = 83.232;
    int angleOffset = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    public PathChain MOVETOLAUNCH;
    public PathChain MOVETOCOLLECT;
    public PathChain COLLECT11;
    public PathChain COLLECT12;
    public PathChain COLLECT13;
    public PathChain MOVETOLAUNCH2;
    public PathChain ENDOFFLINE;
    private boolean ran = false;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        follower.update(); // Update Pedro Pathing

        ch = new Chassis(hardwareMap);

        collectorFront = new Shooter(hardwareMap,"collectorFront", false);

        collectorBack = new Shooter(hardwareMap,"collectorBack", false);

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);

        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap,telemetry);

        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);

        timer2.reset();
        do {
            while (timer2.seconds() < 0.25) {
                limelight.setTeam(false);
                limelight.processRobotPose(telemetry);
                telemetry.addData("x", limelight.getX());
                telemetry.addData("y", limelight.getY());
                telemetry.addData("team ID", limelight.getTeam());
                telemetry.addData("pipeline", limelight.getPipeline());
                telemetry.update();
            }
            limelight.readObelisk(telemetry);
            GlobalStorage.setPattern(limelight.getObelisk());

            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", limelight.getTeam());
            telemetry.addData("Testing Mode", testingMode);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("collectorPower", Shooter.collectorPower);
            telemetry.update();

            if (gamepad1.bWasPressed()) {
                //goalTag.targetAprilTagID = 24;
                limelight.teamID = 24;
                angleOffset = 0;
                autoPoses.build(0,1,line1Y, angleOffset);
                follower.setStartingPose(autoPoses.startPose);
                buildPaths();
                GlobalStorage.setAlliance(24);
            } else if (gamepad1.xWasPressed()) {
                //goalTag.targetAprilTagID = 20;
                limelight.teamID = 20;
                angleOffset = 180;
                autoPoses.build(99.07,-1,line1Y, angleOffset);
                follower.setStartingPose(autoPoses.startPose);
                buildPaths();
                GlobalStorage.setAlliance(20);
            } else if (gamepad1.yWasPressed()) {
                startDelay += 2;
            } else if (gamepad1.aWasPressed()) {
                startDelay -= 1;
            } else if (gamepad1.leftStickButtonWasPressed()) {
                testingMode = true;
            }
            if (limelight.getTeam() == 24 && !ran) {
                angleOffset = 0;
                autoPoses.build(0,1,line1Y, angleOffset);
                follower.setStartingPose(autoPoses.startPose);
                buildPaths();
                GlobalStorage.setAlliance(24);
                ran = true;
            } else if (limelight.getTeam() == 20 && !ran) {
                angleOffset = 180;
                autoPoses.build(99.07,-1,line1Y, angleOffset);
                follower.setStartingPose(autoPoses.startPose);
                buildPaths();
                GlobalStorage.setAlliance(20);
                ran = true;
            }

        } while (opModeInInit());
        waitForStart();
        sleep(1000*startDelay);
        setPathState(0);
        limelight.setTeamID();

        collectorBack.setPower(Shooter.collectorPower);
        collectorFront.setPower(Shooter.collectorPower);

        launchFlapLeft.setPosition(Constants.leftFlapDown);
        launchFlapRight.setPosition(Constants.rightFlapDown);

        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
    }
    public void buildPaths() {
        MOVETOLAUNCH = follower
                .pathBuilder()
                .addPath(new BezierLine(autoPoses.startPose, autoPoses.launchPose))
                .setLinearHeadingInterpolation(Math.toRadians(180-angleOffset), Math.toRadians(Math.abs(angleOffset-45)))
                .build();

        MOVETOCOLLECT = follower
                .pathBuilder()
                .addPath(new BezierLine(autoPoses.launchPose, autoPoses.readyPickUp1))
                .setLinearHeadingInterpolation(Math.toRadians(Math.abs(angleOffset-45)), Math.toRadians(angleOffset))
                .build();

        COLLECT11 = follower
                .pathBuilder()
                .addPath(new BezierLine(autoPoses.readyPickUp1, autoPoses.line11))
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        COLLECT12 = follower
                .pathBuilder()
                .addPath(new BezierLine(autoPoses.line11, autoPoses.line12))
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        COLLECT13 = follower
                .pathBuilder()
                .addPath(new BezierLine(autoPoses.line12, autoPoses.line13))
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        MOVETOLAUNCH2 = follower
                .pathBuilder()
                .addPath(new BezierLine(autoPoses.line13, autoPoses.launchPose))
                .setLinearHeadingInterpolation(Math.toRadians(angleOffset), Math.toRadians(Math.abs(angleOffset-40)))
                .build();

        ENDOFFLINE = follower
                .pathBuilder()
                .addPath(new BezierLine(autoPoses.launchPose, autoPoses.endOffLine))
                .setLinearHeadingInterpolation(Math.toRadians(Math.abs(angleOffset-40)), Math.toRadians(270))
                .build();
    }
        public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(MOVETOLAUNCH, Shooter.maxPower, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (!testingMode) {
                        Shooter.fireVolleySorted(limelight,telemetry,flipper,shooterLeft,launchFlapLeft,shooterRight,launchFlapRight, this);
                    }
                    follower.followPath(MOVETOCOLLECT, Shooter.maxPower, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(COLLECT11, Shooter.maxPower, true);
                    timer.reset();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    flipper.setPosition(1);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT12, Shooter.maxPower, true);
                        timer.reset();
                        setPathState(4);
                    }

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    flipper.setPosition(0);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT13, Shooter.maxPower, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {

                    follower.followPath(MOVETOLAUNCH2, Shooter.maxPower,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (!testingMode) {
                        Shooter.fireVolleySorted(limelight,telemetry,flipper,shooterLeft,launchFlapLeft,shooterRight,launchFlapRight, this);
                    }
                    follower.followPath(ENDOFFLINE, Shooter.maxPower, true);
                    setPathState(7);
                }
                break;
            case 7:
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
    public static class Poses {

        public Pose startPose;
        public Pose launchPose;
        public Pose readyPickUp1, line11, line12, line13;
        public Pose endOffLine;

        public void build(double startOffset, int sign, double line1Y, int angleOffset) {

            // Old starting pose parallel to side of field
            startPose = new Pose(121.536-startOffset, 120.5, Math.toRadians(180-angleOffset));
            launchPose = new Pose(startPose.getX() - 35*sign, line1Y);

            readyPickUp1 = new Pose(startPose.getX() - 25*sign, line1Y);
            line11 = new Pose(startPose.getX() - 14.68*sign, line1Y);
            line12 = new Pose(line11.getX() + 5*sign, line1Y);
            line13 = new Pose(line12.getX() + 5*sign, line1Y);

            endOffLine = new Pose(startPose.getX(), 83.232);
        }
    }
}