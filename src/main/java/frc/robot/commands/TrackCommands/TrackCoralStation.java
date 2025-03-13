// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackCoralStation extends Command {
  /** Creates a new TrackCoralStation. */
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

  private int frontRightTarget_ID;
  private int frontLeftTarget_ID;


  public TrackCoralStation(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
    // PID
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
    // Set limits
    // xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_CoralStation, PhotonConstants.xPidMaxOutput_CoralStation);
    // yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_CoralStation, PhotonConstants.yPidMaxOutput_CoralStation);
    // rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_CoralStation, PhotonConstants.rotationPidMaxOutput_CoralStation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = true;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    frontRightTarget_ID = m_PhotonVisionSubsystem.getFrontRightTargetID();
    frontLeftTarget_ID = m_PhotonVisionSubsystem.getFrontLeftTargetID();
    if(m_PhotonVisionSubsystem.hasBackTarget()) {
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_Back();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_CoralStation_Back);
      rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_CoralStation_Back;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_CoralStation_Back);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_CoralStation);
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_Back();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_CoralStation_Back);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_CoralStation_Back;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_CoralStation_Back);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_CoralStation);
      // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_Back();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_CoralStation_Back);
      xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_CoralStation_Back;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_CoralStation_Back);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_CoralStation);
    }else if(m_PhotonVisionSubsystem.hasFrontTarget()) {
      if(m_PhotonVisionSubsystem.hasFrontRightTarget()) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontRight();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_CoralStation_FrontRight);
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_CoralStation_FrontRight;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_CoralStation_FrontRight);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_CoralStation);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_FrontRight();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_CoralStation_FrontRight);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_CoralStation_FrontRight;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_CoralStation_FrontRight);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_CoralStation);
        // X-PID calculations
        xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_FrontRight();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_CoralStation_FrontRight);
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_CoralStation_FrontRight;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_CoralStation_FrontRight);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_CoralStation);
      }
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }

    if((xPidMeasurements == PhotonConstants.xPidSetPoint_CoralStation_FrontRight 
    && yPidMeasurements == PhotonConstants.yPidSetPoint_CoralStation_FrontRight
    && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_CoralStation_FrontRight)
    || (xPidMeasurements == PhotonConstants.xPidSetPoint_CoralStation_Back
    && yPidMeasurements == PhotonConstants.yPidSetPoint_CoralStation_Back
    && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_CoralStation_Back)) {
        LEDConstants.arrivePosition_Base = true;
        LEDConstants.LEDFlag = true;
    }
    // impl
    if(ElevatorConstants.arriveLow == false) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_CoralStation);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_CoralStation);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_CoralStation);
    }
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
    return LEDConstants.arrivePosition_Base;
  }
}
