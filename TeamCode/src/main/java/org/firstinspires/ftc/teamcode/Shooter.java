package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private DcMotorEx shooter;
    public double Kvelo = 0.0243; // power multiplier for rotations per second
    // FeedBack term is Kp (proportional term)
    // Set Kp to zero when tuning the Kvelo term!!
    public double Kp = 0.3;  // no gain in improvement when increasing beyond this

    static final double   COUNTS_PER_REV = 28 ;  // REV HD Hex 1:1 Motor Encoder

    public double targetVelocity = 0;  // rotations per second (max is ~40)
    public static double collectorPower = 0.53;
    public static double maxPower = 0.5;

    public Shooter(HardwareMap hardwareMap, String name, Boolean dir) {
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, name);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // WITH OUT!
        setMotorDirection(dir);

    }
    public void overridePower() {
        double currentVelocity = shooter.getVelocity(AngleUnit.DEGREES)/COUNTS_PER_REV;
        double veloError = targetVelocity - currentVelocity;
        // CONTROLLER:  feedfoward = Kvelo + feedback = Kpos
        double setPower = targetVelocity * Kvelo  + veloError * Kp;
        shooter.setPower(setPower);
    }
    private void setMotorDirection(Boolean dir) {
        //True = forward, false = backwards
        if (dir) {
            shooter.setDirection(DcMotor.Direction.FORWARD);
        } else {
            shooter.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public void setControllerValues(double Kp, double Kvelo) {
        this.Kp = Kp;
        this.Kvelo = Kvelo;
    }

    public void setTargetVelocity(double velo) {
        this.targetVelocity = velo;
    }
    public double getPower() {
        return shooter.getPower();
    }
    public double getVelocity() {
        return shooter.getVelocity(AngleUnit.DEGREES)/COUNTS_PER_REV;
    }
    public boolean atSpeed() {
        if (0.98*targetVelocity < this.getVelocity() && this.getVelocity() < 1.02*targetVelocity) {
            return true;
        } else {
            return false;
        }
    }
    public void setPower(double power) {
        shooter.setPower(power);
    }
    public static void fireVolleySorted(GoalTagLimelight limelight, Telemetry telemetry, Servo flipper, Shooter shooterLeft, Servo launchFlapLeft, Shooter shooterRight, Servo launchFlapRight, LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double velRight = 0;
        double velLeft = 0;
        while (timer.seconds() < 0.25) {
            limelight.process(telemetry);
            velLeft = (limelight.getRange()+100.99)/7.3712;
            velRight = (limelight.getRange()+100.99)/7.3712;
        }
        if (limelight.getObelisk().equals("PGP")) {
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            fireShooterRight(velRight, shooterRight, launchFlapRight);
            flipper.setPosition(1);
            opMode.sleep(1000);
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            flipper.setPosition(0.525);
        } else if (limelight.getObelisk().equals("GPP")) {
            fireShooterRight(velRight, shooterRight, launchFlapRight);
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            opMode.sleep(1000);
            flipper.setPosition(1);
            opMode.sleep(1000);
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            flipper.setPosition(0.525);
        } else if (limelight.getObelisk().equals("PPG")) {
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            opMode.sleep(1000);
            flipper.setPosition(1);
            opMode.sleep(1000);
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            fireShooterRight(velRight, shooterRight, launchFlapRight);
            flipper.setPosition(0.525);
        }
    }
    public static void fireShooterLeft(double velocity, Shooter shooterLeft, Servo launchFlapLeft) {
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
    public static void fireShooterRight(double velocity, Shooter shooterRight, Servo launchFlapRight) {
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
}