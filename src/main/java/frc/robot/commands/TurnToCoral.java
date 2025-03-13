// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToCoral extends Command {
  /** Creates a new TurnToCoral. */
  private final SwerveSubsystem_Kraken m_SwerveSubsystem_Kraken;

  private final PIDController turningController;

  private double rotationPidError;
  private double currentRotation;
  private double rotationPidOutput;
  public TurnToCoral(SwerveSubsystem_Kraken swerveSubsystem_Kraken) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SwerveSubsystem_Kraken = swerveSubsystem_Kraken;

    turningController = new PIDController(0, 0, 0);

    addRequirements(m_SwerveSubsystem_Kraken);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRotation = m_SwerveSubsystem_Kraken.getRotation().getDegrees();
    rotationPidError = Math.abs(60 - m_SwerveSubsystem_Kraken.getRotation().getDegrees());
    currentRotation = (rotationPidError > 0.5) ? currentRotation : 60;
    
    rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);

    m_SwerveSubsystem_Kraken.drive(0, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
