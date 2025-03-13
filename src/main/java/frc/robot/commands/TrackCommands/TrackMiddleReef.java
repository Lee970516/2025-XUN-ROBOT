// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackMiddleReef extends Command {
  /** Creates a new TrackMiddleReef. */
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem_Kraken m_SwerveSubsystem;

  private PIDController rotationPidController;
  private PIDController xPidController;
  private PIDController yPidController;

  private double xMeasure;
  private double yMeasure;
  private double rotationMeasure;

  private double xPidError;
  private double yPidError;
  private double rotationError;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationPidOutput;

  private int fiducialId;

  public TrackMiddleReef(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
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
      
      rotationMeasure = m_PhotonVisionSubsystem.getRotationMeasurements_FrontRight();
      rotationMeasure = MathUtil.applyDeadband(rotationError, 0.5);
      rotationPidOutput = rotationPidController.calculate(rotationMeasure, PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yMeasure = m_PhotonVisionSubsystem.getYMeasurements_FrontRight();
      yMeasure = MathUtil.applyDeadband(yMeasure, 0.05);
      yPidOutput = -yPidController.calculate(yMeasure, PhotonConstants.yPidSetPoint_MiddleReef_FrontRight);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xMeasure = m_PhotonVisionSubsystem.getXMeasurements_FrontRight();
      xMeasure = MathUtil.applyDeadband(xMeasure, 0.05);
      xPidOutput = -xPidController.calculate(xMeasure, PhotonConstants.xPidSetPoint_MiddleReef_FrontRight);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Reef);
    }else if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
      // Rotation-PID calculations
      rotationMeasure = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
      rotationError = Math.abs(rotationMeasure - PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft);
      rotationMeasure = (rotationError > 0.5) ? rotationMeasure : PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft;
      rotationPidOutput = rotationPidController.calculate(rotationMeasure, PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yMeasure = m_PhotonVisionSubsystem.getYMeasurements_FrontLeft();
      yPidError = Math.abs(yMeasure - PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft);
      yMeasure = (yPidError > 0.05) ? yMeasure : PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft;
      yPidOutput = -yPidController.calculate(yMeasure, PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xMeasure = m_PhotonVisionSubsystem.getXMeasurements_FrontLeft();
      xPidError = Math.abs(xMeasure - PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft);
      xMeasure = (xPidError > 0.05) ? xMeasure : PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft;
      xPidOutput = -xPidController.calculate(xMeasure, PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Reef);
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }

    if((xMeasure == PhotonConstants.xPidSetPoint_MiddleReef_FrontRight 
    && yMeasure == PhotonConstants.yPidSetPoint_MiddleReef_FrontRight
    && rotationMeasure == PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight)
    || (xMeasure == PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft
    && yMeasure == PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft
    && rotationMeasure == PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft)) {
        LEDConstants.arrivePosition_Base = true;
        LEDConstants.LEDFlag = true;
      }
    // impl

    SmartDashboard.putBoolean("isFinish", LEDConstants.arrivePosition_Base);
    SmartDashboard.putNumber("TrackMiddle/xPidOutput", xPidOutput);
    SmartDashboard.putNumber("TrackMiddle/yPidOutput", yPidOutput);
    SmartDashboard.putNumber("TrackMiddle/rotationPidOutput", rotationPidOutput);
    SmartDashboard.putNumber("TrackMiddle/xPidError", xPidError);
    SmartDashboard.putNumber("TrackMiddle/yPidError", yPidError);

    if(ElevatorConstants.arriveLow == false) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Reef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Reef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Reef);
    }

    m_SwerveSubsystem.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.arrivePosition_Base = false;
    LEDConstants.tracking = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
