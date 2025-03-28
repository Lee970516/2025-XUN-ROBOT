// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Neo;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackRightReef_Neo extends Command {
  /** Creates a new TrackRightReef_Neo. */
 private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem_Neo m_SwerveSubsystem;

  private PIDController rotationPidController;
  private PIDController xPidController;
  private PIDController yPidController;

  private double xPidMeasurements;
  private double yPidMeasurements;
  private double rotationPidMeasurements;

  private double xPidError;
  private double yPidError;
  private double rotationPidError;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationPidOutput;

  private boolean rotationReady;
  private boolean yReady;

  private final DoubleSupplier xSpeedFunc;
  private final SlewRateLimiter xLimiter;

  private double xSpeed;
  private boolean photonGood;

  public TrackRightReef_Neo(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Neo swerveSubsystem, DoubleSupplier xSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    xSpeedFunc = xSpeed;
    photonGood = false;

    xLimiter = new SlewRateLimiter(4.6);

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
    // PID
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
    // Set limits
    // xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_Reef, PhotonConstants.xPidMaxOutput_Reef);
    // yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_Reef, PhotonConstants.yPidMaxOutput_Reef);
    // rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_Reef, PhotonConstants.rotationPidMaxOutput_Reef);
    // rotationPidController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.drive(0, 0, 0, false);

    rotationReady = false;
    yReady = false;

    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = MathUtil.applyDeadband(xSpeed, OperatorConstants.kJoystickDeadBand);
    if (ElevatorConstants.arriveLow) {
      xSpeed = xSpeedFunc.getAsDouble() * 0.6;
    }else{
      xSpeed = xSpeedFunc.getAsDouble() * 0.2;
    }
    xSpeed = xLimiter.calculate(xSpeed);

    if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_RightReef);
      rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_RightReef;
      if(rotationPidError < 0.5) {
        rotationReady = true;
      }
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_RightReef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_FrontLeft();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_RightReef);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_RightReef;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_RightReef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      if(yPidError < 0.05) {
        yReady = true;
      }
      // X-PID calculations
      if(rotationReady && yReady) {
      xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_FrontLeft();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_RightReef);
      xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_RightReef;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_RightReef);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Reef);
      }else {
        xPidOutput = 0;
      }
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }
    // impl
    if(yPidMeasurements == PhotonConstants.yPidSetPoint_RightReef 
    && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_RightReef) {
      LEDConstants.arrivePosition_Base = true;
      LEDConstants.LEDFlag = true;
      photonGood = true;
    }else {
      photonGood = false;
    }

    SmartDashboard.putNumber("TrackRightReef/xPidOutput", xPidOutput);
    SmartDashboard.putNumber("TrackRightReef/yPidOutput", yPidOutput);
    SmartDashboard.putNumber("TrackRightReef/rotationPidOutput", rotationPidOutput);
    SmartDashboard.putNumber("TrackRightReef/xPidError", xPidError);
    SmartDashboard.putNumber("TrackRightReef/yPidError", yPidError);
    SmartDashboard.putNumber("TrackRightReef/rotationPidError", rotationPidError);
    
    SmartDashboard.putBoolean("Photon/RightPhotonGood", photonGood);
    m_SwerveSubsystem.drive(xSpeed, yPidOutput, rotationPidOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0, false);
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.tracking = false;
    LEDConstants.LEDFlag = true;
    photonGood = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
