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

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Constants;
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
    Servo lift;

    // Timers
    ElapsedTime timerLeft = new ElapsedTime();
    ElapsedTime timerRight = new ElapsedTime();
    ElapsedTime timerFlipper = new ElapsedTime();

    Chassis ch;
    private double idlePower = 20;
    private double kP = 0.3; // was 0.14 before adding 0 breaking
    private boolean leftIsRunning;
    private boolean rightIsRunning;
    private boolean shootSequence;

    private boolean shootSquenceStep1 = true;

    private boolean shootSquenceStep2 = false;
    private boolean emergencyMode = false;
    private boolean slowChildMode = false;

    @Override
    public void init() {
        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");
        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");
        lift = hardwareMap.get(Servo.class, "lift");
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
            limelight.teamID = GlobalStorage.getAlliance();
        }
        timerLeft.reset();
        timerRight.reset();
        timerFlipper.reset();


    }

    @Override
    public void init_loop() {
        telemetry.addData("Pattern", limelight.getObelisk());
        telemetry.addData("team ID", limelight.getID());

        telemetry.addLine("Bumpers to shoot, a to turntotag");
        telemetry.addLine("Press b for red, x for blue");
        telemetry.update();
        if (gamepad1.bWasPressed()) {
            limelight.teamID = 24;
            limelight.setTeamID();
        } else if (gamepad1.xWasPressed()) {
            limelight.teamID = 20;
            limelight.setTeamID();
        } else if (gamepad1.rightStickButtonWasPressed()) {
            slowChildMode = true;
        }
    }

    @Override
    public void start() {
        collectorFront.setPower(Shooter.collectorPower);
        collectorBack.setPower(Shooter.collectorPower);
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

        if (gamepad1.leftBumperWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
            // do math here
            //shooterLeft.targetVelocity = (limelight.getRange() + 202.17 - 10) / 8.92124;
            //shooterLeft.targetVelocity = (limelight.getRange()+100.99)/7.3712;
            if (!emergencyMode) {
                shooterLeft.targetVelocity = shooterRight.getShooterVelo(limelight);
            }
            leftIsRunning = true;
            timerLeft.reset();
        }
        if (gamepad1.rightBumperWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
            // do math here
            //shooterRight.targetVelocity = (limelight.getRange() + 202.17 - 10) / 8.92124;
            //shooterRight.targetVelocity = (limelight.getRange()+100.99)/7.3712;
            if (!emergencyMode) {
                shooterRight.targetVelocity = shooterRight.getShooterVelo(limelight);
            }
            rightIsRunning = true;
            timerRight.reset();
        }
        if (gamepad1.yWasPressed()) {
            idlePower = 0;
            lift.setPosition(Constants.liftRetract);
            // turn everything off
            collectorBack.setPower(0);
            collectorFront.setPower(0);
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            launchFlapLeft.setPosition(Constants.leftFlapDown);
            launchFlapRight.setPosition(Constants.rightFlapDown);
        }
        if (gamepad1.bWasPressed()) {
            lift.setPosition(Constants.liftExtend);
            telemetry.addData("Lift Value", lift.getPosition());
        }
        if (gamepad1.dpadLeftWasPressed()) {
            flipper.setPosition(1);
            timerFlipper.reset();
        }
        if (gamepad1.dpadRightWasPressed()) {
            flipper.setPosition(0.1);
            timerFlipper.reset();
        }
        if (gamepad1.dpadUpWasPressed()) {
            collectorBack.setPower(-Shooter.collectorPower);
            collectorFront.setPower(-Shooter.collectorPower);
        }
        if (gamepad1.dpadUpWasReleased()) {
            collectorFront.setPower(Shooter.collectorPower);
            collectorBack.setPower(Shooter.collectorPower);
        }
        if (gamepad1.a && limelight.isDataCurrent) {
            ch.turnTo(limelight.getTx(), 0);
        }

        // Parking mode
        if (gamepad1.right_trigger == 1 || slowChildMode) {
            ch.setMaxSpeed(0.5);
        } else {
            ch.setMaxSpeed(1);
        }
        ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.left_trigger == 1 && limelight.isDataCurrent) {
            shootSequence = true;
        }
        if(shootSequence) {
            ElapsedTime timer = new ElapsedTime();
            if(shootSquenceStep1) {
                shooterLeft.targetVelocity = shooterLeft.getShooterVelo(limelight);
                shooterRight.targetVelocity = shooterRight.getShooterVelo(limelight);
                leftIsRunning = true;
                rightIsRunning = true;
                timerLeft.reset();
                timerRight.reset();
                shootSquenceStep1 = false;
                shootSquenceStep2 = true;
            }
            if(!(leftIsRunning || rightIsRunning) && timer.seconds() > 2 && shootSquenceStep2) {
                flipper.setPosition(1);
                timerFlipper.reset();
                shooterLeft.targetVelocity = shooterLeft.getShooterVelo(limelight);
                timerLeft.reset();
                launchFlapLeft.setPosition(Constants.leftFlapUp);
                leftIsRunning = true;
                shootSquenceStep2 = false;
                shootSequence = false;
            }

        }
        // Shoot when at speed
        if (leftIsRunning) {
            if (shooterLeft.atSpeed()) {
                timerLeft.reset();
                launchFlapLeft.setPosition(Constants.leftFlapUp);
                leftIsRunning = false;
            }
        }
        if (rightIsRunning) {
            if (shooterRight.atSpeed()) {
                timerRight.reset();
                launchFlapRight.setPosition(Constants.rightFlapUp);
                rightIsRunning = false;
            }
        }
        // Servo Reset
        if (timerLeft.seconds() > 0.6 && !leftIsRunning) {
            launchFlapLeft.setPosition(Constants.leftFlapDown);
            shooterLeft.targetVelocity = idlePower;
        }
        if (timerRight.seconds() > 0.6 && !rightIsRunning) {
            launchFlapRight.setPosition(Constants.rightFlapDown);
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
}
