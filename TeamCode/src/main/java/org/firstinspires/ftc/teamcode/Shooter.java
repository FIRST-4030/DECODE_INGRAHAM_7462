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
    public static double collectorPower = 0.6; //0.53
    public static double maxPower = 1.0;

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
        while (timer.seconds() < 0.1) {
            limelight.process(telemetry);
            velLeft = shooterLeft.getShooterVelo(limelight);
            velRight = shooterRight.getShooterVelo(limelight);
        }
        if (limelight.getObelisk().equals("PGP")) {
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            fireShooterRight(velRight, shooterRight, launchFlapRight);
            flipper.setPosition(1);
            opMode.sleep(500);
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            flipper.setPosition(0.525);
        } else if (limelight.getObelisk().equals("GPP")) {
            fireShooterRight(velRight, shooterRight, launchFlapRight);
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            flipper.setPosition(1);
            opMode.sleep(500);
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            flipper.setPosition(0.525);
        } else if (limelight.getObelisk().equals("PPG")) {
            fireShooterLeft(velLeft, shooterLeft, launchFlapLeft);
            flipper.setPosition(1);
            opMode.sleep(500);
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
        launchFlapLeft.setPosition(Constants.leftFlapUp);
        while (timer.seconds() < 0.8) {
            shooterLeft.overridePower();
        }
        launchFlapLeft.setPosition(Constants.leftFlapDown);
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
        launchFlapRight.setPosition(Constants.rightFlapUp);
        while (timer.seconds() < 0.8) {
            shooterRight.overridePower();
        }
        launchFlapRight.setPosition(Constants.rightFlapDown);
        while (timer.seconds() < 1) {
            shooterRight.overridePower();
        }
    }
    public double getShooterVelo(GoalTagLimelight limelight) {
        // compute velocity from range using function based on shooting experiments
        double range = limelight.getRange();
        if (range < 80) {
            double poly = 29;
            return poly;
        } else {
            double poly = 19 + 0.125*range;
            return poly;
        }
        //26.2 - 0.0381*range + 0.000915*range*range; // 2nd order polynomial
        //return (limelight.getRange() + 202.17 - 10) / 8.92124; // older function
    }

}