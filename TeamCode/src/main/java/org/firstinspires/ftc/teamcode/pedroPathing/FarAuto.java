package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.onbotjava.OnBotJavaTracingStandardFileManager;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Shooter;


@Autonomous(name = "FarAuto", group = "Autonomous")
@Configurable // Panels
public class FarAuto extends LinearOpMode {

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
    Datalog datalogFar;

    private int startDelay = 0;
    private boolean testingMode = false;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    Poses autoPoses = new Poses();
    private int angleOffset;


    public PathChain MOVETOLAUNCH;
    public PathChain PREPARETOCOLLECT1;
    public PathChain COLLECT11;
    public PathChain COLLECT12;
    public PathChain COLLECT13;
    public PathChain MOVETOLAUNCH2;
    public PathChain PREPARETOCOLLECT2;
    public PathChain COLLECT21;
    public PathChain COLLECT22;
    public PathChain COLLECT23;
    public PathChain MOVETOLAUNCH3;
    public PathChain ENDOFFLINE;

    final double line1Y = 35.478;
    final double line2Y = 59.340;
    final double flipperDelay = 1.0;
    private boolean ran = false;
    ElapsedTime runtime = new ElapsedTime();
    boolean logDataFar = false;

    public String stepName = "void";

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

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

        timer2.reset();
        do {
            while (timer2.seconds() < 3) {
                limelight.setTeam(true);
                limelight.processRobotPose(telemetry);
                telemetry.addData("x", limelight.getX());
                telemetry.addData("y", limelight.getY());
                telemetry.addData("team ID", limelight.getTeam());
                telemetry.addData("pipeline", limelight.getPipeline());
                telemetry.update();
            }
            limelight.readObelisk(telemetry);
            GlobalStorage.setPattern(limelight.getObelisk());

            telemetry.addData("start",autoPoses.startPose);
            telemetry.addData("launch",autoPoses.launchPose);
            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", limelight.getTeam());
            telemetry.addData("Testing Mode", testingMode);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("pipeline", limelight.getPipeline());
            telemetry.addData("collectorPower", Shooter.collectorPower);
            telemetry.update();

            if (gamepad1.bWasPressed()) {
                limelight.teamID = 24;
                angleOffset = 0;
                autoPoses.build(0,1,line1Y,line2Y);
                follower.setStartingPose(autoPoses.startPose);
                buildPaths();
                GlobalStorage.setAlliance(24);
            } else if (gamepad1.xWasPressed()) {
                limelight.teamID = 20;
                angleOffset = 180;
                autoPoses.build(20,-1,line1Y,line2Y);
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
         else if (gamepad1.rightStickButtonWasPressed()) {
            logDataFar = true;
        }

            if (limelight.getTeam() == 24 && !ran) {
                angleOffset = 0;
                autoPoses.build(0,1,line1Y,line2Y);
                follower.setStartingPose(autoPoses.startPose);
                buildPaths();
                ran = true;
                GlobalStorage.setAlliance(24);
            } else if (limelight.getTeam() == 20 && !ran) {
                angleOffset = 180;
                autoPoses.build(20,-1,line1Y,line2Y);
                follower.setStartingPose(autoPoses.startPose);
                buildPaths();
                GlobalStorage.setAlliance(20);
                ran = true;
            }
        } while (opModeInInit());

        if (logDataFar) {
            datalogFar = new FarAuto.Datalog("FarAuto");
        }

        waitForStart();
        sleep(1000*startDelay);
        setPathState(0);
        limelight.setTeamID();

        collectorBack.setPower(Shooter.collectorPower);
        collectorFront.setPower(Shooter.collectorPower);

        launchFlapLeft.setPosition(0.3);
        launchFlapRight.setPosition(0.4);

        while (opModeIsActive()) {
            limelight.processRobotPose(telemetry);
            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine

            telemetry.addData("x", limelight.getX());
            telemetry.addData("y", limelight.getY());
            telemetry.addData("Tx", limelight.getTx());

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
    }

    public void buildPaths() {

        MOVETOLAUNCH = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.startPose, autoPoses.launchPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(Math.abs(angleOffset-71)))
                .build();

        PREPARETOCOLLECT1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.launchPose, autoPoses.readyPickUp1)
                )
                .setLinearHeadingInterpolation(Math.toRadians(Math.abs(angleOffset-71)), Math.toRadians(angleOffset))
                .build();

        COLLECT11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.readyPickUp1, autoPoses.line11)
                )
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        COLLECT12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.line11, autoPoses.line12)
                )
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        COLLECT13 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.line12, autoPoses.line13)
                )
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        MOVETOLAUNCH2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.line13, autoPoses.launchPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(angleOffset), Math.toRadians(Math.abs(angleOffset-65)))
                .build();

        PREPARETOCOLLECT2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.launchPose, autoPoses.readyPickUp2)
                )
                .setLinearHeadingInterpolation(Math.toRadians(Math.abs(angleOffset-65)), Math.toRadians(angleOffset))
                .build();

        COLLECT21 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.readyPickUp2, autoPoses.line21)
                )
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        COLLECT22 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.line21, autoPoses.line22)
                )
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        COLLECT23 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.line22, autoPoses.line23)
                )
                .setConstantHeadingInterpolation(Math.toRadians(angleOffset))
                .build();

        MOVETOLAUNCH3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.line23, autoPoses.launchPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(angleOffset), Math.toRadians(Math.abs(angleOffset-67)))
                .build();

        ENDOFFLINE = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoPoses.launchPose, autoPoses.endOffLine)
                )
                .setConstantHeadingInterpolation(Math.toRadians(Math.abs(angleOffset-67)))
                .build();
    }
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(MOVETOLAUNCH, Shooter.maxPower, true);
                stepName = "MOVETOLAUNCH";
                if (logDataFar) { logOneSample(follower.getPose()); }
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (Math.abs(limelight.getTx()) > 1) {
                        ch.turnTo(limelight.getTx(), 0);
                    } else {
                        if (!testingMode) {
                            Shooter.fireVolleySorted(limelight, telemetry, flipper, shooterLeft, launchFlapLeft, shooterRight, launchFlapRight, this);
                        }
                        follower.followPath(PREPARETOCOLLECT1, Shooter.maxPower, true);
                        stepName = "PREPARETOCOLLECT1";
                        if (logDataFar) {
                            logOneSample(follower.getPose());
                        }
                        setPathState(2);
                        //}
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(COLLECT11, Shooter.maxPower, true);
                    timer.reset();
                    stepName = "PREPARETOCOLLECT11";
                    if (logDataFar) { logOneSample(follower.getPose()); }
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    flipper.setPosition(Constants.flipperRight);
                    if (timer.seconds() > flipperDelay) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT12, Shooter.maxPower, true);
                        timer.reset();
                        stepName = "PREPARETOCOLLECT12";
                        if (logDataFar) { logOneSample(follower.getPose()); }
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    flipper.setPosition(Constants.flipperLeft);
                    if (timer.seconds() > flipperDelay) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT13, Shooter.maxPower, true);
                        stepName = "PREPARETOCOLLECT13";
                        if (logDataFar) { logOneSample(follower.getPose()); }
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {

                    follower.followPath(MOVETOLAUNCH2, Shooter.maxPower,true);
                    stepName = "MOVETOLAUNCH2";
                    if (logDataFar) { logOneSample(follower.getPose()); }
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (!testingMode) {
                        Shooter.fireVolleySorted(limelight,telemetry,flipper,shooterLeft,launchFlapLeft,shooterRight,launchFlapRight, this);
                    }
                    follower.followPath(PREPARETOCOLLECT2, Shooter.maxPower, true);
                    stepName = "PREPARETOCOLLECT2";
                    if (logDataFar) { logOneSample(follower.getPose()); }
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(COLLECT21, Shooter.maxPower, true);
                    timer.reset();
                    stepName = "COLLECT21";
                    if (logDataFar) { logOneSample(follower.getPose()); }
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    flipper.setPosition(Constants.flipperLeft);
                    if (timer.seconds() > flipperDelay) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT22, Shooter.maxPower, true);
                        timer.reset();

                        stepName = "COLLECT22";
                        if (logDataFar) { logOneSample(follower.getPose()); }
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    flipper.setPosition(Constants.flipperRight);
                    if (timer.seconds() > flipperDelay) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT23, Shooter.maxPower, true);
                        stepName = "COLLECT23";
                        if (logDataFar) { logOneSample(follower.getPose()); }
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(MOVETOLAUNCH3, Shooter.maxPower, true);
                    stepName = "MOVETOLAUNCH3";
                    if (logDataFar) { logOneSample(follower.getPose()); }
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if (!testingMode) {
                        Shooter.fireVolleySorted(limelight,telemetry,flipper,shooterLeft,launchFlapLeft,shooterRight,launchFlapRight, this);
                    }
                    follower.followPath(ENDOFFLINE, Shooter.maxPower, true);
                    stepName = "ENDOFFLINE";
                    if (logDataFar) { logOneSample(follower.getPose()); }
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    stepName = "UNDEFINEDPATH";
                    if (logDataFar) { logOneSample(follower.getPose()); }
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
        public Pose readyPickUp2, line21, line22, line23;
        public Pose endOffLine;

        public void build(double startOffset, int sign, double line1Y, double line2Y) {

            startPose = new Pose(81.850-startOffset, 8.348, Math.toRadians(90));
            launchPose = new Pose(startPose.getX()+5*sign, 15.324);

            readyPickUp1 = new Pose(startPose.getX()+20*sign, line1Y);
            line11 = new Pose(startPose.getX()+25*sign, line1Y);
            line12 = new Pose(line11.getX() + 5*sign, line1Y);
            line13 = new Pose(line12.getX() + 5*sign, line1Y);

            readyPickUp2 = new Pose(startPose.getX()+20*sign, line2Y);
            line21 = new Pose(startPose.getX()+25*sign, line2Y);
            line22 = new Pose(line21.getX() + 5*sign, line2Y);
            line23 = new Pose(line22.getX() + 5*sign, line2Y);

            endOffLine = new Pose(startPose.getX()+10*sign, 26);
        }
    }
    private void logOneSample(Pose pose) {
        datalogFar.runTime.set(runtime.seconds());
        datalogFar.xPose.set(pose.getX());
        datalogFar.yPose.set(pose.getY());
        datalogFar.heading.set(pose.getHeading());
        datalogFar.tx.set(limelight.getTx());
        datalogFar.ty.set(limelight.getTy());
        datalogFar.dataName.set(stepName);
        datalogFar.writeLine();
    }
    public static class Datalog {
        /*
         * The underlying datalogger object - it cares only about an array of loggable fields
         */
        private final Datalogger datalogger;
        /*
         * These are all of the fields that we want in the datalog.
         * Note: Order here is NOT important. The order is important
         *       in the setFields() call below
         */
        public Datalogger.GenericField runTime = new Datalogger.GenericField("runTime");
        public Datalogger.GenericField xPose   = new Datalogger.GenericField("X");
        public Datalogger.GenericField yPose   = new Datalogger.GenericField("Y");
        public Datalogger.GenericField heading = new Datalogger.GenericField("Heading");
        public Datalogger.GenericField tx = new Datalogger.GenericField("tx");
        public Datalogger.GenericField ty = new Datalogger.GenericField("ty");
        public Datalogger.GenericField dataName = new Datalogger.GenericField("dataName");

        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    /*
                     * Tell it about the fields we care to log.
                     * Note: Order *IS* important here! The order in which we list the
                     *       fields is the order in which they will appear in the log.
                     */
                    .setFields( runTime, xPose, yPose, heading, tx, ty, dataName)
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}
