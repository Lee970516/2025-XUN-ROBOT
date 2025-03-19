// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants.LEDConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae extends Command {
  /** Creates a new PrimitiveIntake_Algae. */
  private final EndEffectorSubsystem m_EndEffectorSubsystem;
  private final ElevatorSubsystem m_ElevatorSubsystem;

  public RemoveAlgae(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_EndEffectorSubsystem.hasCoral() == false) {
      m_EndEffectorSubsystem.RemoveAlgae();
      m_EndEffectorSubsystem.Arm_RemoveAlgae();
      m_ElevatorSubsystem.toPrimitive();

      LEDConstants.intakeArriving = false;
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.hasGamePiece = true;
      LEDConstants.LEDFlag = true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffectorSubsystem.hasCoral() == false && m_ElevatorSubsystem.arrivePrimition()) {
      m_EndEffectorSubsystem.Arm_IDLE();
    }
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
