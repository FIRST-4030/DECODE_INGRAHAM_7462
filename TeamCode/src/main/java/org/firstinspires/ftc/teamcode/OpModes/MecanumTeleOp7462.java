/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Shooter;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Mecanum TeleOp 7462", group = "Robot")
//@Disabled //comment this out when ready to add to android phone
public class MecanumTeleOp7462 extends OpMode {
    GoalTagLimelight limelight;
    Shooter collectorBack;
    Shooter collectorFront;
    Shooter shooterLeft;
    Shooter shooterRight;

    Servo launchFlapLeft;
    Servo launchFlapRight;
    Servo flipper;

    // Timers
    ElapsedTime timerLeft = new ElapsedTime();
    ElapsedTime timerRight = new ElapsedTime();
    ElapsedTime timerFlipper = new ElapsedTime();

    Chassis ch;
    private double idlePower = 20;
    private double kP = 0.3; // was 0.14 before adding 0 breaking
    private boolean leftIsRunning;
    private boolean rightIsRunning;
    private boolean emergencyMode = false;
    private boolean slowChildMode = false;

    @Override
    public void init() {
        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");
        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");
        flipper = hardwareMap.get(Servo.class, "flipper");

        ch = new Chassis(hardwareMap);

        collectorFront = new Shooter(hardwareMap, "collectorFront", false);
        collectorBack = new Shooter(hardwareMap, "collectorBack", false);

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterLeft.setControllerValues(0.3, 0.0243);

        shooterRight = new Shooter(hardwareMap, "shooterRight", false);
        shooterRight.setControllerValues(0.3, 0.0243);

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap, telemetry);

        if ((GlobalStorage.getAlliance() != -1)) {
            limelight.setPipeline(GlobalStorage.getAlliance());
        }
        timerLeft.reset();
        timerRight.reset();
        timerFlipper.reset();


    }

    //we are using the methods from OpMode and @Override is so that we can write our own stuff for this method
// Move to auto
    @Override
    public void init_loop() {
        telemetry.addData("Pattern", limelight.getObelisk());
        telemetry.addData("team ID", limelight.getID());

        telemetry.addLine("Bumpers to shoot, a to turntotag");
        telemetry.addLine("Press b for red, x for blue");
        telemetry.update();
        if (gamepad1.bWasPressed()) {
//            goalTag.targetAprilTagID = 24;
            limelight.teamID = 24;
            limelight.setTeamID();
        } else if (gamepad1.xWasPressed()) {
            //goalTag.targetAprilTagID = 20;
            limelight.teamID = 20;
            limelight.setTeamID();
        } else if (gamepad1.rightStickButtonWasPressed()) {
            slowChildMode = true;
        }
        // remove later
//        else if (gamepad1.yWasPressed()) {
//            kP += 0.01;
//        } else if (gamepad1.aWasPressed()) {
//            kP -= 0.01;
//        }
    }

    @Override
    public void start() {
        collectorFront.setPower(0.6);
        collectorBack.setPower(0.6);
    }

    @Override
    public void loop() {
        limelight.process(telemetry);

        shooterRight.overridePower();
        shooterLeft.overridePower();


        telemetry.addData("shooterLeftCurrentVelocity", shooterLeft.getVelocity());
        telemetry.addData("shooterLeftTargetVelocity", shooterLeft.targetVelocity);
        telemetry.addData("shooterRightCurrentVelocity", shooterRight.getVelocity());
        telemetry.addData("shooterRightTargetVelocity", shooterRight.targetVelocity);
        telemetry.addData("collectorFrontCurrentPower", collectorFront.getPower());
        telemetry.addData("collectorBackCurrentPower", collectorBack.getPower());
        telemetry.addData("Kp", kP);
        telemetry.addData("TimerLeft", timerLeft.seconds());
        telemetry.update();


        // Driver Controlstelemetry.addData("Is Tag Recent", limelight.seeObelisk);
        if (gamepad1.leftBumperWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
            // do math here
            //shooterLeft.targetVelocity = (limelight.getRange() + 202.17 - 10) / 8.92124;
            //shooterLeft.targetVelocity = (limelight.getRange()+100.99)/7.3712;
            shooterLeft.targetVelocity = shooterLeft.getShooterVelo(limelight);
            leftIsRunning = true;
            timerLeft.reset();
        }
        if (gamepad1.rightBumperWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
            // do math here
            //shooterRight.targetVelocity = (limelight.getRange() + 202.17 - 10) / 8.92124;
            //shooterRight.targetVelocity = (limelight.getRange()+100.99)/7.3712;
            shooterRight.targetVelocity = shooterRight.getShooterVelo(limelight);
            rightIsRunning = true;
            timerRight.reset();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            flipper.setPosition(1);
            timerFlipper.reset();
        }
        if (gamepad2.dpadRightWasPressed()) {
            flipper.setPosition(0.1);
            timerFlipper.reset();
        }
        if (gamepad2.dpadUpWasPressed()) {
            collectorBack.setPower(-0.6);
            collectorFront.setPower(-0.6);
        }
        if (gamepad2.dpadUpWasReleased()) {
            collectorFront.setPower(0.6);
            collectorBack.setPower(0.6);
        }
        if (gamepad1.a && limelight.isDataCurrent) {
            ch.turnTo(limelight.getTx(), 0);
        }
//        if (gamepad2.yWasPressed()) {
//            collectorPower += 0.05;
//
//        } else if (gamepad2.aWasPressed()) {
//            collectorPower -= 0.05;

        // Parking mode
        if (gamepad1.right_trigger == 1 || slowChildMode) {
            ch.setMaxSpeed(0.5);
        } else {
            ch.setMaxSpeed(1);
        }
        ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        if (gamepad1.yWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
//            shooterLeft.targetVelocity = (limelight.getRange()+100.99)/7.3712;
//            shooterRight.targetVelocity = (limelight.getRange()+100.99)/7.3712;
//            leftIsRunning = true;
//            rightIsRunning = true;
//            timerLeft.reset();
//            timerRight.reset();
//            flipper.setPosition(1);
//            timerFlipper.reset();
//            shooterLeft.targetVelocity = (limelight.getRange()+100.99)/7.3712;
//            timerLeft.reset();
//            leftIsRunning = true;
//        }
        // Shoot when at speed
        if (leftIsRunning) {
            if (shooterLeft.atSpeed()) {
                timerLeft.reset();
                launchFlapLeft.setPosition(0);
                leftIsRunning = false;
            }
        }
        if (rightIsRunning) {
            if (shooterRight.atSpeed()) {
                timerRight.reset();
                launchFlapRight.setPosition(0.7);
                rightIsRunning = false;
            }
        }
        // Servo Reset
        if (timerLeft.seconds() > 0.5 && !leftIsRunning) {
            launchFlapLeft.setPosition(0.3);
            shooterLeft.targetVelocity = idlePower;
        }
        if (timerRight.seconds() > 0.5 && !rightIsRunning) {
            launchFlapRight.setPosition(0.4);
            shooterRight.targetVelocity = idlePower;
        }
        if (timerFlipper.seconds() > 0.25) {
            flipper.setPosition(0.525);
        }
        // If camera not working press this and shoot near point of close V.
        if (gamepad1.leftStickButtonWasPressed()) {
            shooterLeft.targetVelocity = 30;
            shooterRight.targetVelocity = 30;
            idlePower = 30;
            emergencyMode = true;
        }
    }
//    public void turnToAprilTagLimelight() {
//        if (limelight.getRange() < 100) {
//            turnTo(0.25, 0.5);
//        } else {
//            if (limelight.getID() == 20) {
//                turnTo(0.25, 2);
//            } else if (limelight.getID() == 24) {
//                turnTo(0.25, -2);
//            }
//        }
//    }

}
