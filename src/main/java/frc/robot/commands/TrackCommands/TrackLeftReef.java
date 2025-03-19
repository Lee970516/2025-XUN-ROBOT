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
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackLeftReef extends Command {
  /** Creates a new TrackReef. */
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem_Kraken m_SwerveSubsystem;

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

  private int fiducialId;

  public TrackLeftReef(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
    // PID
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
    // Set limits
    // xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_Reef, PhotonConstants.xPidMaxOutput_Reef);
    // yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_Reef, PhotonConstants.yPidMaxOutput_Reef);
    // rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_Reef, PhotonConstants.rotationPidMaxOutput_Reef);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_PhotonVisionSubsystem.hasFrontRightTarget()) {
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontRight();
      rotationPidError = m_PhotonVisionSubsystem.getRotationError_Reef("LeftReef");
      rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_LeftReef;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_LeftReef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_FrontRight();
      yPidError = m_PhotonVisionSubsystem.getYError_Reef("LeftReef");
      yPidMeasurements = (yPidError > 0.02) ? yPidMeasurements : PhotonConstants.yPidSetPoint_LeftReef;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_LeftReef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_FrontRight();
      xPidError = m_PhotonVisionSubsystem.getRotationError_Reef("LeftReef");
      xPidMeasurements = (xPidError > 0.02) ? xPidMeasurements : PhotonConstants.xPidSetPoint_LeftReef;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_LeftReef);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidSetPoint_LeftReef);
      // if(xPidController.getError()<0.05) SmartDashboard.putBoolean("Align/LeftReefAlign", true);
      // else SmartDashboard.putBoolean("Align/LeftReefAlign", false);

      if(m_PhotonVisionSubsystem.isArrive_Reef("LeftReef")) {
        LEDConstants.arrivePosition_Base = true;
        LEDConstants.LEDFlag = true;
      }
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }
    // LED control 
    // Speed limit protection
    if(ElevatorConstants.arriveLow == false) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Reef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Reef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Reef);
    }
    SmartDashboard.putNumber("Photon/RotationPidOutput", rotationPidOutput);
    // Impl
    m_SwerveSubsystem.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.tracking = false;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
