// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve_KrakenConstants;

public class SwerveSubsystem_Kraken extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveModule_Kraken leftFront;
  private final SwerveModule_Kraken leftBack;
  private final SwerveModule_Kraken rightFront;
  private final SwerveModule_Kraken rightBack;

  private final Pigeon2 gyro;
  private final Pigeon2Configuration gyroConfig;

  private final SwerveDriveOdometry odometry;

  private final Field2d field;

  private RobotConfig robotConfig;
  
  /**
   * 
   */
  public SwerveSubsystem_Kraken() {
    leftFront = new SwerveModule_Kraken(
      Swerve_KrakenConstants.leftFrontTurning_ID,
      Swerve_KrakenConstants.leftFrontDrive_ID,
      Swerve_KrakenConstants.leftFrontAbsolutedEncoder_ID,
      Swerve_KrakenConstants.leftFrontOffset
            );
    rightFront = new SwerveModule_Kraken(
      Swerve_KrakenConstants.rightFrontTurning_ID,
      Swerve_KrakenConstants.rightFrontDrive_ID,
      Swerve_KrakenConstants.rightFrontAbsolutedEncoder_ID,
      Swerve_KrakenConstants.rightFrontOffset);
    leftBack = new SwerveModule_Kraken(
      Swerve_KrakenConstants.leftBackTurning_ID,
      Swerve_KrakenConstants.leftBackDrive_ID,
      Swerve_KrakenConstants.leftBackAbsolutedEncoder_ID,
      Swerve_KrakenConstants.leftBackOffset);
    rightBack = new SwerveModule_Kraken(
      Swerve_KrakenConstants.rightBackTurning_ID,
      Swerve_KrakenConstants.rightBackDrive_ID,
      Swerve_KrakenConstants.rightBackAbsolutedEncoder_ID,
      Swerve_KrakenConstants.rightBackOffset);

     gyro = new Pigeon2(Swerve_KrakenConstants.gyro_ID);
     gyroConfig = new Pigeon2Configuration();

     gyroConfig.MountPose.MountPoseYaw = 0;
     gyroConfig.MountPose.MountPosePitch = 0;
     gyroConfig.MountPose.MountPoseRoll = 0;

     gyro.getConfigurator().apply(gyroConfig);

     field = new Field2d();

     odometry = new SwerveDriveOdometry(Swerve_KrakenConstants.swerveKinematics, getRotation(), getModulesPosition(), getRobotPose());

     resetGyro();
     // All other subsystem initialization
    // ...                                                              
    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getRobotPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(Swerve_KrakenConstants.pathingtheta_Kp, Swerve_KrakenConstants.pathingtheta_Ki, Swerve_KrakenConstants.pathingtheta_Kd), // Translation PID constants
                    new PIDConstants(Swerve_KrakenConstants.pathingMoving_Kp, Swerve_KrakenConstants.pathingMoving_Ki, Swerve_KrakenConstants.pathingMoving_Kd) // Rotation PID constants
            ),
            robotConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    // // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

  }


  public ChassisSpeeds getChassisSpeed() {
    return Swerve_KrakenConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }


  public Pose2d getRobotPose() {
    return field.getRobotPose();
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public SwerveModulePosition[] getModulesPosition() {
    return new SwerveModulePosition[]{
      leftFront.getPosition(),
      rightFront.getPosition(),
      leftBack.getPosition(),
      rightBack.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      leftFront.getState(),
      rightFront.getState(),
      leftBack.getState(),
      rightBack.getState()
    };
  }

  public void setModouleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond);
      leftFront.setState(desiredStates[0]);
      rightFront.setState(desiredStates[1]);
      leftBack.setState(desiredStates[2]);
      rightBack.setState(desiredStates[3]);
  }

  public void setModouleStates_Auto(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond);
    leftFront.setState(desiredStates[0]);
    rightFront.setState(desiredStates[1]);
    leftBack.setState(desiredStates[2]);
    rightBack.setState(desiredStates[3]);
}

  public void resetGyro() {
    gyro.reset();
  }

  public void setPose(Pose2d poses) {
    odometry.resetPosition(getRotation(), getModulesPosition(), poses);
  }


  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient) {
    SwerveModuleState[] state;
    xSpeed = xSpeed * Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond;
    ySpeed = ySpeed * Swerve_KrakenConstants.maxDriveSpeed_MeterPerSecond;
    zSpeed = zSpeed * Math.toRadians(Swerve_KrakenConstants.maxAngularVelocity_Angle);;
    if(fieldOrient) {
      state = Swerve_KrakenConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation()));//之後要處理MaxSpeedPerSecond跟MaxRadianPerSecond的問題
    }else{
      state = Swerve_KrakenConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }
    setModouleStates(state);
  } 

  public void autoDrive(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.01);
    SwerveModuleState[] states = Swerve_KrakenConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);

    setModouleStates(states);
  }

  public void stopMotor(){
    leftBack.stopMotor();
    leftFront.stopMotor();
    rightBack.stopMotor();
    rightFront.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation(), getModulesPosition());
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("Swerve/Yaw", gyro.getYaw().getValueAsDouble());

    SmartDashboard.putNumber("Swerve/gyro", getRotation().getDegrees());
    SmartDashboard.putNumber("Swerve/leftFrontAbsolutePosion", leftFront.getTurningPosition());
    SmartDashboard.putNumber("Swerve/leftBackAbsolutePosion", leftBack.getTurningPosition());
    SmartDashboard.putNumber("Swerve/rightFrontAbsolutePosion", rightFront.getTurningPosition());
    SmartDashboard.putNumber("Swerve/rightBackAbsolutePosion", rightBack.getTurningPosition());

    SmartDashboard.putNumber("Swerve/leftFrontTurningMotorPosition", leftFront.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/leftBackTurningMotorPosition", leftBack.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/rightFrontTurningMotorPosition", rightFront.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/rightBackTurningMotorPosition", rightBack.getTurningMotorPosition());

    SmartDashboard.putNumber("Swerve/rightFrontTurningSetpoint", leftFront.getStateAngle());
    
    SmartDashboard.putNumber("Swerve/leftFrontDrivingMotorPosition", leftFront.getDrivePosition());
    SmartDashboard.putNumber("Swerve/leftBackDrivingMotorPosition", leftBack.getDrivePosition());
    SmartDashboard.putNumber("Swerve/rightFrontDrivingMotorPosition", rightFront.getDrivePosition());
    SmartDashboard.putNumber("Swerve/rightBackDrivingMotorPosition", rightBack.getDrivePosition());
    
    SmartDashboard.putNumber("Swerve/leftFrontStateAngle", leftFront.getStateAngle());
    SmartDashboard.putNumber("Swerve/leftFrontAngle", leftFront.getTurningAngle());

    SmartDashboard.putNumber("Swerve/GyroYaw", gyro.getYaw().getValueAsDouble());
  }
}
