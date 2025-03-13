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
public class TrackCage extends Command {
  /** Creates a new TrackCage. */
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

  private int backTarget_ID;

  public TrackCage(SwerveSubsystem_Kraken swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
    // PID
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
    // Set limits
    // xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_Cage, PhotonConstants.xPidMaxOutput_Cage);
    // yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_Cage, PhotonConstants.yPidMaxOutput_Cage);
    // rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_Cage, PhotonConstants.rotationPidMaxOutput_Cage);
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
    backTarget_ID = m_PhotonVisionSubsystem.getBackTargetID();

    if(m_PhotonVisionSubsystem.hasFrontTarget()) {
      if(m_PhotonVisionSubsystem.hasFrontRightTarget()) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontRight();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_FrontRight);
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_FrontRight;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_FrontRight);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Cage);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_FrontRight();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_FrontRight);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_FrontRight;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_FrontRight);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Cage);
        // X-PID calculations
        xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_FrontRight();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_FrontRight);
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_Cage_FrontRight;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_FrontRight);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Cage);
      }else if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_FrontLeft);
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_FrontLeft;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_FrontLeft);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Cage);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_FrontLeft();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_FrontLeft);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_FrontLeft;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_FrontLeft);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Cage);
        // X-PID calculations
        xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_FrontLeft();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_FrontLeft);
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_Cage_FrontLeft;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_FrontLeft);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Cage);
    }
  }else if(m_PhotonVisionSubsystem.hasBackTarget()) {
    if(backTarget_ID == 20 || backTarget_ID == 11) {
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_Back();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_Back_ID20_ID11);
      rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_Back_ID20_ID11;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_Back_ID20_ID11);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Cage);
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_Back();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_Back_ID20_ID11);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_Back_ID20_ID11;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_Back_ID20_ID11);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Cage);
      // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_Back();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_Back_ID20_ID11);
      xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_Cage_Back_ID20_ID11;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_Back_ID20_ID11);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Cage);
    }else if(backTarget_ID == 21 || backTarget_ID == 10) {
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_Back();
      rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_Cage_Back_ID21_ID10);
      rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Cage_Back_ID21_ID10;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Cage_Back_ID21_ID10);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Cage);
      // Y-PID calculations
      yPidMeasurements = m_PhotonVisionSubsystem.getYMeasurements_Back();
      yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_Cage_Back_ID21_ID10);
      yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Cage_Back_ID21_ID10;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Cage_Back_ID21_ID10);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Cage);
      // X-PID calculations
      xPidMeasurements = m_PhotonVisionSubsystem.getXMeasurements_Back();
      xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_Cage_Back_ID21_ID10);
      xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_Cage_Back_ID21_ID10;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Cage_Back_ID21_ID10);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Cage);
    }
  }else {
    xPidOutput = 0;
    yPidOutput = 0;
    rotationPidOutput = 0;
  }
  if((xPidMeasurements == PhotonConstants.xPidSetPoint_Cage_FrontRight 
  && yPidMeasurements == PhotonConstants.yPidSetPoint_Cage_FrontRight
  && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_Cage_FrontRight)
  || (xPidMeasurements == PhotonConstants.xPidSetPoint_Cage_FrontLeft
  && yPidMeasurements == PhotonConstants.yPidSetPoint_Cage_FrontLeft
  && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_Cage_FrontLeft)
  || (xPidMeasurements == PhotonConstants.xPidSetPoint_Cage_Back_ID21_ID10
  && yPidMeasurements == PhotonConstants.yPidSetPoint_Cage_Back_ID21_ID10
  && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_Cage_Back_ID21_ID10)
  || (xPidMeasurements == PhotonConstants.xPidSetPoint_Cage_Back_ID20_ID11
  && yPidMeasurements == PhotonConstants.yPidSetPoint_Cage_Back_ID20_ID11
  && rotationPidMeasurements == PhotonConstants.rotationPidSetPoint_Cage_Back_ID20_ID11)) {
      LEDConstants.LEDFlag = true;
      LEDConstants.arrivePosition_Base = true;
      }
    // impl
    if(ElevatorConstants.arriveLow == false) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Cage);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Cage);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Cage);
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
