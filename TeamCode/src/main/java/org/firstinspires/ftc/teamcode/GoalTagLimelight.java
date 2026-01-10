package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


public class GoalTagLimelight {
    Limelight3A limelight;
    private double goalYaw; // inches
    private double goalRange; // in

    public int teamID;

    public boolean GPP = false; // id 21
    public boolean PGP = false; // id 22
    public boolean PPG = false; // id 23
    public boolean seeObelisk = false;
    private double tx;
    private double ty;
    private double x;
    private double y;

    private double camera_height = 16.4; // in
    private double target_height = 29.5; // in
    private double camera_angle = 0.032; // radians old was 0.0418

    public boolean isDataCurrent;
    IMU imu;
    //Pipeline 5 is 20(blue) pipeline 1 is 24(red)

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(50);
        limelight.start(); // This tells Limelight to start looking!
    }

    public void readObelisk(Telemetry telemetry) {
        limelight.pipelineSwitch(6); //targets closest
        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            seeObelisk = false;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            if (tagId == 21) {
                telemetry.addData("Detected", "Tag ID 21");
                PGP = false;
                PPG = false;
                GPP = true;
                seeObelisk = true;
            } else if (tagId == 22) {
                telemetry.addData("Detected", "Tag ID 22");
                PGP = true;
                PPG = false;
                GPP = false;
                seeObelisk = true;
            } else if (tagId == 23) {
                telemetry.addData("Detected", "Tag ID 23");
                PGP = false;
                PPG = true;
                GPP = false;
                seeObelisk = true;
            }
        }
    }
    public void processRobotPose(Telemetry telemetry) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();
            x = botPose.getPosition().x;
            y = botPose.getPosition().y;
            double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);
            Pose pedroPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.METER, x, y, AngleUnit.DEGREES, heading), InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            telemetry.addData("Pose", pedroPose);
        }
    }

    public void process(Telemetry telemetry) {

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();

            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)

            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            if (botPose != null) {
//                x = botPose.getPosition().x;
//                telemetry.addData("botx", x);
//                y = botPose.getPosition().y;
//                telemetry.addData("boty", y);

                goalYaw = botPose.getOrientation().getYaw();
                goalRange = (target_height - camera_height) / (Math.tan(Math.toRadians(ty)+camera_angle));

                isDataCurrent = true;

                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            } else {
                isDataCurrent = false;
            }

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("pipeline", result.getPipelineIndex());
            telemetry.addData("limelight Range", goalRange);
            telemetry.addData("Target Area", ta);
        } else {
            isDataCurrent = false;
            telemetry.addData("Limelight", "No Targets");
        }
    }

    public void setTeam(boolean far) {
        limelight.pipelineSwitch(7); // Show both goals
        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (far) {
                teamID = fiducial.getFiducialId();
            } else {
                if (fiducial.getFiducialId() == 20) {
                    teamID = 24;
                } else if (fiducial.getFiducialId() == 24) {
                    teamID = 20;
                }
            }

        }
    }
    public void setTeamID() {
        if (teamID == 20) {
            limelight.pipelineSwitch(5);
        } else if (teamID == 24) {
            limelight.pipelineSwitch(1);
        }
    }
    public void setPipeline(int id) {
        limelight.pipelineSwitch(id);
        teamID = id;
    }
    public String getObelisk() {
        if (PGP) {
            return "PGP";
        } else if (GPP) {
            return "GPP";
        } else if (PPG) {
            return "PPG";
        } else {
            return "No Tag Detected";
        }
    }
    public int getPipeline() {return limelight.getStatus().getPipelineIndex();}
    public int getTeam() {
      return teamID;
    }
    public double getX()
    {
        return x;
    }
    public double getY()
    {
        return y;
    }
    public double getRange() {
        return goalRange;
    }
    public double getTx() {
        return tx;
    }
    public int getID() {
        return teamID;
    }
}
