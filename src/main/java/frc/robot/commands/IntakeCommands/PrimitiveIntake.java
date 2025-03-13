// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrimitiveIntake extends Command {
  /** Creates a new PrimitiveIntake. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  public PrimitiveIntake(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffectorSubsystem.hasAlgae()) {
      m_EndEffectorSubsystem.Arm_IDLE();
      m_EndEffectorSubsystem.holdAlgae();
      if(m_EndEffectorSubsystem.arrivedSetpoint()) {
        m_ElevatorSubsystem.toPrimitive();
      }

      LEDConstants.intakeArriving = false;
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.hasGamePiece = true;
      LEDConstants.LEDFlag = true;
    }else if(m_EndEffectorSubsystem.hasCoral() && Math.abs(m_ElevatorSubsystem.getCurrentPosition() - ElevatorConstants.coralL4Position) <= 2) {
      m_EndEffectorSubsystem.primitiveArm_HasCoral();
      m_EndEffectorSubsystem.stopWheel();
      if(m_EndEffectorSubsystem.arrivedSetpoint()) {
        m_ElevatorSubsystem.toPrimitive();
        if(m_ElevatorSubsystem.arriveSetPoint()) {
          m_EndEffectorSubsystem.Arm_IDLE();

          LEDConstants.intakeArriving = false;
          LEDConstants.arrivePosition_Intake = false;
          LEDConstants.LEDFlag = true;
        }
      }
    }else {
      m_EndEffectorSubsystem.Arm_IDLE();
      m_EndEffectorSubsystem.stopWheel();
      if(m_EndEffectorSubsystem.arrivedSetpoint()) {
        m_ElevatorSubsystem.toPrimitive();
      }

      LEDConstants.intakeArriving = false;
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
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
