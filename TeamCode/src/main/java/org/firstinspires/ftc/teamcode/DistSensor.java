package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DistSensor {
    private DistanceSensor middleSensor;
    private DistanceSensor leftSensor;
    private DistanceSensor rightSensor;
    private boolean ballLeft = false;
    private boolean ballRight = false;
    private boolean ballMiddle = false;

    ElapsedTime sensorTimer = new ElapsedTime();
    ElapsedTime flipDelay = new ElapsedTime();

    private RunningAverage leftDist = new RunningAverage(5);
    private RunningAverage midDist = new RunningAverage(5);
    private RunningAverage rightDist = new RunningAverage(5);

    public void init(HardwareMap hardwareMap) {
        middleSensor = hardwareMap.get(DistanceSensor.class, "middleSensor");
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
    }

    public void loop(Servo flipper, ElapsedTime timerFlipper, ElapsedTime timerLeft, ElapsedTime timerRight, Shooter collectorBack, Shooter collectorFront, Gamepad gamepad1, boolean manualFlip) {
        if (sensorTimer.milliseconds() > 100) {

            leftDist.addNumber(Math.max(6, leftSensor.getDistance(DistanceUnit.INCH)));
            midDist.addNumber(Math.max(6, middleSensor.getDistance(DistanceUnit.INCH)));
            rightDist.addNumber(Math.max(6, rightSensor.getDistance(DistanceUnit.INCH)));

            sensorTimer.reset();
        }

        if (leftDist.getAverage() < 6.5) {
            ballLeft = true;
        } else {
            ballLeft = false;
        }

        if (midDist.getAverage() < 6.5) {
            ballMiddle = true;
        } else {
            ballMiddle = false;
        }

        if (rightDist.getAverage() < 6.5) {
            ballRight = true;
        } else {
            ballRight = false;
        }

        if (!manualFlip) {
            if (!ballLeft && ballMiddle && timerLeft.seconds() > 0.5) {
                flipper.setPosition(Constants.flipperLeft);
                timerFlipper.reset();
                flipDelay.reset();
            } else if (!ballRight && ballMiddle && flipDelay.seconds() > 1 && timerRight.seconds() > 0.5) {
                flipper.setPosition(Constants.flipperRight);
                timerFlipper.reset();
            } else if (ballLeft && ballRight && ballMiddle) {
                if (!gamepad1.dpad_up) {
                    collectorBack.setPower(0);
                    collectorFront.setPower(0);
                }
            } else {
                if (!gamepad1.dpad_up) {
                    collectorBack.setPower(Shooter.collectorPower);
                    collectorFront.setPower(Shooter.collectorPower);
                }
            }
        }
    }
}
