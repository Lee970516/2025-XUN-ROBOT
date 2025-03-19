// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */

  private final PhotonCamera frontRightCamera;
  private final PhotonCamera frontLeftCamera;
  private final PhotonCamera backCamera;

  private final Transform3d robotToFrontRight;
  private final Transform3d robotToFrontLeft;
  private final Transform3d robotToBack;


  private PhotonPipelineResult frontRightResult;
  private PhotonPipelineResult frontLeftResult;
  private PhotonPipelineResult backResult;
  private PhotonTrackedTarget frontRightTarget;
  private PhotonTrackedTarget frontLeftTarget;
  private PhotonTrackedTarget backTarget;
  private Optional<MultiTargetPNPResult> results;
  private List<PhotonTrackedTarget> frontRightTargets;
  private List<PhotonTrackedTarget> frontLeftTargets;
  private List<PhotonTrackedTarget> backTargets;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private int frontRightTarget_ID;
  private int frontLeftTarget_ID;
  private int backTarget_ID;


  private double botXMeasurements_FrontRight;
  private double botYMeasurements_FrontRight;
  private double botRotationMeasurements_FrontRight;
  private double botXMeasurements_FrontLeft;
  private double botYMeasurements_FrontLeft;
  private double botRotationMeasurements_FrontLeft;
  private double botXMeasurements_Back;
  private double botYMeasurements_Back;
  private double botRotationMeasurements_Back;


  public PhotonVisionSubsystem() {
    frontRightCamera = new PhotonCamera("OV9281_FrontRight");
    frontLeftCamera = new PhotonCamera("OV9281_FrontLeft");
    backCamera = new PhotonCamera("OV9281_Back");

    robotToFrontRight = new Transform3d(0.2555, -0.221, 0.21, new Rotation3d(0, 0, Math.toRadians(-48.964)));
    robotToFrontLeft = new Transform3d(-0.2555, -0.221, 0.21, new Rotation3d(new Rotation2d(Units.degreesToRadians(48.964))));
    robotToBack = new Transform3d(0, 0, 0, new Rotation3d(new Rotation2d(0)));

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  }

  public int getFrontRightTargetID() {
    return frontRightTarget_ID;
  }

  public int getFrontLeftTargetID() {
    return frontLeftTarget_ID;
  }

  public int getBackTargetID() {
    return backTarget_ID;
  }

  public boolean hasFrontRightTarget() {
    return frontRightResult.hasTargets();
  }

  public boolean hasFrontLeftTarget() {
    return frontLeftResult.hasTargets();
  }

  public boolean hasBackTarget() {
    return backResult.hasTargets();
  }

  public boolean hasFrontTarget() {
    if(hasFrontRightTarget() || hasFrontLeftTarget()) return true;
    return false;
  }

  public boolean hasTarget() {
    if(hasFrontTarget() || hasBackTarget()) return true;
    return false;
  }

  public Transform3d getFrontRightTargetPose() {
    return frontRightTarget.getBestCameraToTarget();
  }

  public Transform3d getFrontLeftTargetPose() {
    return frontLeftTarget.getBestCameraToTarget();
  }

  public Transform3d getBackTargetPose() {
    return backTarget.getBestCameraToTarget();
  }

  public Transform3d getRobotToTargetPose_FrontRight() {
    // return frontRightTarget.getBestCameraToTarget().plus(frontRightToRobot);
    return frontRightTarget.getBestCameraToTarget();
  }

  public Transform3d getRobotToTargetPose_FrontLeft() {
    return frontLeftTarget.getBestCameraToTarget();
  }

  public Transform3d getRobotToTargetPose_Back() {
    return backTarget.getBestCameraToTarget();
  }

  public Optional<Matrix<N3, N3>> getCameraMatrix(String camera) {
    if(camera == "FrontRight") return frontRightCamera.getCameraMatrix();
    if(camera == "FrontLeft") return frontLeftCamera.getCameraMatrix();
    if(camera == "Back") return backCamera.getCameraMatrix();
    return null;
  }

  public Optional<Matrix<N8, N1>> getCameraDistCoeffs(String camera) {
    if(camera == "FrontRight") return frontRightCamera.getDistCoeffs();
    if(camera == "FrontLeft") return frontLeftCamera.getDistCoeffs();
    if(camera == "Back") return backCamera.getDistCoeffs();
    return null;
  }

  public double getXMeasurements_FrontRight() {
    return botXMeasurements_FrontRight;
  }

  public double getYMeasurements_FrontRight() {
    return botYMeasurements_FrontRight;
  }

  public double getRotationMeasurements_FrontRight() {
    return botRotationMeasurements_FrontRight;
  }

  public double getXMeasurements_FrontLeft() {
    return botXMeasurements_FrontLeft;
  }

  public double getYMeasurements_FrontLeft() {
    return botYMeasurements_FrontLeft;
  }

  public double getRotationMeasurements_FrontLeft() {
    return botRotationMeasurements_FrontLeft;
  }

  public double getXMeasurements_Back() {
    return botXMeasurements_Back;
  }

  public double getYMeasurements_Back() {
    return botYMeasurements_Back;
  }

  public double getRotationMeasurements_Back() {
    return botRotationMeasurements_Back;
  }
  
  public double getXError_Reef(String reef) {
    if(reef == "RightReef") return Math.abs(getXMeasurements_FrontLeft() - PhotonConstants.xPidSetPoint_RightReef);
    else if(reef == "LeftReef") return Math.abs(getXMeasurements_FrontRight() - PhotonConstants.xPidSetPoint_LeftReef);
    else if(reef == "MiddleReef_FrontRight") return Math.abs(getXMeasurements_FrontRight() - PhotonConstants.xPidSetPoint_MiddleReef_FrontRight);
    else return Math.abs(getXMeasurements_FrontLeft() - PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft);
  }

  public double getYError_Reef(String reef) {
    if(reef == "RightReef") return Math.abs(getYMeasurements_FrontLeft() - PhotonConstants.yPidSetPoint_RightReef);
    else if(reef == "LeftReef") return Math.abs(getYMeasurements_FrontRight() - PhotonConstants.yPidSetPoint_LeftReef);
    else if(reef == "MiddleReef_FrontRight") return Math.abs(getYMeasurements_FrontRight() - PhotonConstants.yPidSetPoint_MiddleReef_FrontRight);
    else return Math.abs(getYMeasurements_FrontLeft() - PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft);
  }

  public double getRotationError_Reef(String reef) {
    if(reef == "RightReef") return Math.abs(getRotationMeasurements_FrontLeft() - PhotonConstants.rotationPidSetPoint_RightReef);
    else if(reef == "LeftReef") return Math.abs(getRotationMeasurements_FrontRight() - PhotonConstants.rotationPidSetPoint_LeftReef);
    else if(reef == "MiddleReef_FrontRight") return Math.abs(getRotationMeasurements_FrontRight() - PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight);
    else return Math.abs(getRotationMeasurements_FrontLeft() - PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft);
  }

  public boolean isArrive_Reef(String reef) {
    if(reef == "rightReef") {
      if((getXError_Reef("RightReef")) <= 0.03 && (getYError_Reef("RightReef") <= 0.03) && (getRotationError_Reef("RightReef") <= 0.75) && hasFrontLeftTarget()) return true;
      else return false;
    }else if(reef == "LeftReef") {
      if((getXError_Reef("LeftReef")) <= 0.02 && (getYError_Reef("LeftReef") <= 0.02) && (getRotationError_Reef("LeftReef") <= 0.5) && hasFrontRightTarget()) return true;
      else return false;
    }else if(reef == "MiddleReef_FrontRight") {
      if((getXError_Reef("MiddleReef_FrontRight") <= 0.02) && (getYError_Reef("MiddleReef_FrontRight") <= 0.02) && (getRotationError_Reef("MiddleReef_FrontRight") <= 0.5)) return true;
      else return false;
    }else {
      if((getXError_Reef("MiddleReef_FrontLeft") <= 0.02) && (getYError_Reef("MiddleReef_FrontLeft") <= 0.02) && (getRotationError_Reef("MiddleReef_FrontLeft") <= 0.5)) return true;
      else return false;
    } 
  }

  public PhotonPipelineResult getResult(String camera) {
    if(camera == "FrontRight") return frontRightResult;
    if(camera == "FrontLeft") return frontLeftResult;
    if(camera == "Back") return backResult;
    return null;
  }

  public static Translation3d rotateTranslation(Translation3d translation, double yaw) {
    double cosYaw = Math.cos(yaw);
    double sinYaw = Math.sin(yaw);

    // 旋轉 X, Y 坐標
    double newX = translation.getX() * cosYaw - translation.getY() * sinYaw;
    double newY = translation.getX() * sinYaw + translation.getY() * cosYaw;
    double newZ = translation.getZ(); // Z 不受影響

    return new Translation3d(newX, newY, newZ);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontRightResult = frontRightCamera.getLatestResult();
    frontRightTarget = frontRightResult.getBestTarget();
    // frontRightTargets = frontRightResult.getTargets();
    frontLeftResult = frontLeftCamera.getLatestResult();
    frontLeftTarget = frontLeftResult.getBestTarget();
    // frontLeftTargets = frontLeftResult.getTargets();
    backResult = backCamera.getLatestResult();
    backTarget = backResult.getBestTarget();
    // backTargets = backResult.getTargets();
    if(hasFrontRightTarget()) {
      botXMeasurements_FrontRight = getRobotToTargetPose_FrontRight().getTranslation().getX();
      botYMeasurements_FrontRight = getRobotToTargetPose_FrontRight().getTranslation().getY();
      botRotationMeasurements_FrontRight = Math.toDegrees(getRobotToTargetPose_FrontRight().getRotation().getAngle());

      // Translation3d botTranslation_FrontRight = getRobotToTargetPose_FrontRight().getTranslation();
      // Rotation3d botRotation_FrontRight = getRobotToTargetPose_FrontRight().getRotation();
      // Pose3d CameraToTag_FrontRight = new Pose3d(botTranslation_FrontRight, botRotation_FrontRight);
      // Translation3d robotToFrontRight_Translation3d = robotToFrontRight.getTranslation();
      // Rotation3d robotToFrontRight_Rotation3d = robotToFrontRight.getRotation();
      // Pose3d robotToFrontRightPose = new Pose3d(robotToFrontRight_Translation3d, robotToFrontRight_Rotation3d);

      // int frontRightTag_ID = frontRightTarget.getFiducialId();

      // Pose3d tagPose = aprilTagFieldLayout.getTagPose(frontRightTag_ID).get();

      // Pose3d robotPose_FrontRight = robotToFrontRightPose.transformBy(getFrontRightTargetPose());

      // botXMeasurements_FrontRight = robotPose_FrontRight.getTranslation().getX();
      // botYMeasurements_FrontRight = robotPose_FrontRight.getTranslation().getY();
      // botRotationMeasurements_FrontRight = Math.toDegrees(robotPose_FrontRight.getRotation().getAngle());


      // Translation3d rotatedTranslation = rotateTranslation(getRobotToTargetPose_FrontRight().getTranslation(), robotToFrontRight.getRotation().getZ());
      // Translation3d robotToTagTranslation = rotatedTranslation.plus(robotToFrontRight.getTranslation());
      // Rotation3d robotToTagRotation = new Rotation3d(
      // getRobotToTargetPose_FrontRight().getRotation().getX() + robotToFrontRight.getRotation().getX(),
      // getRobotToTargetPose_FrontRight().getRotation().getY() + robotToFrontRight.getRotation().getY(),
      // getRobotToTargetPose_FrontRight().getRotation().getZ() + robotToFrontRight.getRotation().getZ()
      // );

      // botXMeasurements_FrontRight = robotToTagTranslation.getX();
      // botYMeasurements_FrontRight = robotToTagTranslation.getY();
      // botRotationMeasurements_FrontRight = Math.toDegrees(robotToTagRotation.getAngle());
      

      frontRightTarget_ID = frontRightTarget.getFiducialId();
      

      SmartDashboard.putNumber("Photon/botXMeasurements_FrontRight", botXMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/botYMeasurements_FrontRight", botYMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/botRotationMeasurements_FrontRight", botRotationMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/FrontRightTarget_ID", frontRightTarget_ID);
      SmartDashboard.putNumber("Photon/botXError_FrontRight", getXError_Reef("FrontRight"));
      SmartDashboard.putNumber("Photon/botYError_FrontRight", getYError_Reef("FrontRight"));
      SmartDashboard.putNumber("Photon/botRotationError_FrontRight", getRotationError_Reef("FrontRight"));

    }else {
      botXMeasurements_FrontRight = 0;
      botYMeasurements_FrontRight = 0;
      botRotationMeasurements_FrontRight = 0;
    }
    if(hasFrontLeftTarget()) {
      botXMeasurements_FrontLeft = getRobotToTargetPose_FrontLeft().getX();
      botYMeasurements_FrontLeft = getRobotToTargetPose_FrontLeft().getY();
      botRotationMeasurements_FrontLeft = Math.toDegrees(getRobotToTargetPose_FrontLeft().getRotation().getAngle());

      frontLeftTarget_ID = frontLeftTarget.getFiducialId();
      

      SmartDashboard.putNumber("Photon/BotXError_FrontLeft", getXError_Reef("FrontLeft"));
      SmartDashboard.putNumber("Photon/BotYError_FrontLeft", getYError_Reef("FrontLeft"));
      SmartDashboard.putNumber("Photon/BotRotationError_FrontLeft", getRotationError_Reef("FrontLeft"));
      SmartDashboard.putNumber("Photon/FrontLeftTarget_ID", frontLeftTarget_ID);

    }else {
      botXMeasurements_FrontLeft = 0;
      botYMeasurements_FrontLeft = 0;
      botRotationMeasurements_FrontLeft = 0;
    }
    if(hasBackTarget()) {
      botXMeasurements_Back = getRobotToTargetPose_Back().getX();
      botYMeasurements_Back = getRobotToTargetPose_Back().getY();
      botRotationMeasurements_Back = Math.toDegrees(getRobotToTargetPose_Back().getRotation().getAngle());

      backTarget_ID = backTarget.getFiducialId();
      

      // SmartDashboard.putNumber("Photon/BotXError_Front", botXMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/BotYError_Front", botYMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/BotRotationError_Front", botRotationMeasurements_FrontRight);
      // SmartDashboard.putNumber("Photon/FrontTarget_ID", frontRightTarget_ID);

    }else {
      botXMeasurements_Back = 0;
      botYMeasurements_Back = 0;
      botRotationMeasurements_Back = 0;
      backTarget_ID = 0;
    }
    if(LEDConstants.hasGamePiece && hasFrontRightTarget()) {
      LEDConstants.canTrackLeft = true;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.canTrackLeft = false;
      LEDConstants.LEDFlag = true;
    }
    if(LEDConstants.hasGamePiece && hasFrontLeftTarget()) {
      LEDConstants.canTrackRight = true;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.canTrackRight = false;
      LEDConstants.LEDFlag = true;
    }   
  }
}
