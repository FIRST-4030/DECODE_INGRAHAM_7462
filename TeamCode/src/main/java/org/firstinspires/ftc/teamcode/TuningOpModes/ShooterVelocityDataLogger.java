package org.firstinspires.ftc.teamcode.TuningOpModes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@TeleOp(name = "ShooterDataLogger")
public class ShooterVelocityDataLogger extends OpMode {

    Servo launchFlapLeft;
    Servo launchFlapRight;
    GoalTagLimelight limelight;
    private double initPos = 0.5;

    private double goalRange;
    private double goalBearing;

    Chassis ch;
    Datalog AimTestDatalog; // create the data logger object

    private boolean goal;

    private int i = 0; // loop counter

    private int k = 0;
    Shooter shooterLeft;
    Shooter shooterRight;

    private boolean readyToShoot = false;


    @Override
    public void init() {

        //shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterLeft.setControllerValues(0.3, 0.0243);

        shooterRight = new Shooter(hardwareMap, "shooterRight", false);
        shooterRight.setControllerValues(0.3, 0.0243);
        ch = new Chassis(hardwareMap);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");
        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");
        //launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap, telemetry);

        // Initialize the datalog
        AimTestDatalog = new Datalog("launch log");
        // wait for start command

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
    }
    public void init_loop() {
        if (gamepad1.bWasPressed()) {
//            goalTag.targetAprilTagID = 24;
            limelight.teamID = 24;
            limelight.setTeamID();
        } else if (gamepad1.xWasPressed()) {
            limelight.teamID = 20;
            limelight.setTeamID();
        }
        telemetry.addData("Pattern", limelight.getObelisk());
        telemetry.addData("team ID", limelight.getID());
        telemetry.update();
        shooterLeft.targetVelocity = 26;
        shooterRight.targetVelocity = 26;
    }
    public void loop() {
        i++;
        k++;
        shooterLeft.overridePower();
        shooterRight.overridePower();

        limelight.process(telemetry);

        goalRange = limelight.getRange();
        double targetVelocity = shooterLeft.targetVelocity;

        if (gamepad1.leftBumperWasPressed()) {
            goal = true;
            telemetry.addData("goal", goal);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.update();
            AimTestDatalog.goalBool.set(goal);
            AimTestDatalog.targetPower.set(targetVelocity);
            AimTestDatalog.goalRange.set(goalRange);
            AimTestDatalog.goalBearing.set(goalBearing);
            AimTestDatalog.writeLine();
        } else if (gamepad1.rightBumperWasPressed()) {
            goal = false;
            telemetry.addData("goal", goal);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.update();
            AimTestDatalog.goalBool.set(goal);
            AimTestDatalog.targetPower.set(targetVelocity);
            AimTestDatalog.goalRange.set(goalRange);
            AimTestDatalog.goalBearing.set(goalBearing);
            AimTestDatalog.writeLine();
        } else if (gamepad1.yWasPressed()) {
            shooterLeft.targetVelocity += 0.5;
            shooterRight.targetVelocity += 0.5;
        } else if (gamepad1.aWasPressed()) {
            shooterLeft.targetVelocity -= 0.5;
            shooterRight.targetVelocity -= 0.5;
        } else if (gamepad1.left_trigger == 1) {
            launchFlapLeft.setPosition(0);
            i = 0;
            readyToShoot = false;
        } else if (gamepad1.right_trigger == 1) {
            launchFlapRight.setPosition(0.7);
            k = 0;
            readyToShoot = false;
        }
        if (i > 500) {
            launchFlapLeft.setPosition(0.3);
            i = 0;
        }
        if (k > 500) {
            launchFlapRight.setPosition(0.4);
            k = 0;
        }
        ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.b && limelight.isDataCurrent) {
            ch.turnTo(limelight.getTx(), 0);
        }

        telemetry.addData("val", gamepad1.right_trigger);


        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("GoalRange", (goalRange));
        telemetry.addData("GoalBearing", (goalBearing));
        telemetry.update();
    }

    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField targetPower = new Datalogger.GenericField("targetVelocity");
        public Datalogger.GenericField goalBool = new Datalogger.GenericField("goalBool");
        public Datalogger.GenericField goalRange = new Datalogger.GenericField("goalRange");
        public Datalogger.GenericField goalBearing = new Datalogger.GenericField("goalBearing");

        public Datalog(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            goalBool,
                            targetPower,
                            goalRange,
                            goalBearing
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.

        public void writeLine() {
            datalogger.writeLine();
        }
    }
}


