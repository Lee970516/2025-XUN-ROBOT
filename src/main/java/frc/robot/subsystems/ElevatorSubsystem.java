// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX elevator_FirstMotor;
  private final TalonFX elevator_SecondMotor;

  private final TalonFXConfiguration elevatorConfig;
  private final Slot0Configs elevatorSlot0Config;
  private final MotionMagicConfigs elevatorMotionMagicConfig;

  private final MotionMagicVoltage request_Elevator;

  private double goalPosition;
  private double lastPosition;
  
  private boolean ifChange_High;
  private boolean ifChange_Low;

  public ElevatorSubsystem() {
    elevator_FirstMotor = new TalonFX(ElevatorConstants.elevator_FirstMotor_ID);
    elevator_SecondMotor = new TalonFX(ElevatorConstants.elevator_SecondMotor_ID);
    elevator_SecondMotor.setControl(new Follower(ElevatorConstants.elevator_FirstMotor_ID, true));
    goalPosition = ElevatorConstants.primitivePosition;

    // Motor Configurations
    elevatorConfig = new TalonFXConfiguration();
    elevatorSlot0Config = new Slot0Configs();
    elevatorMotionMagicConfig = new MotionMagicConfigs();
    request_Elevator = new MotionMagicVoltage(goalPosition);

    elevatorSlot0Config.kS = 0;
    elevatorSlot0Config.kG = 0.44;
    elevatorSlot0Config.kV = 0;
    elevatorSlot0Config.kA = 0;
    elevatorSlot0Config.kP = 0.9;
    elevatorSlot0Config.kI = 0;
    elevatorSlot0Config.kD = 0;

    elevatorMotionMagicConfig.MotionMagicCruiseVelocity = 40;
    elevatorMotionMagicConfig.MotionMagicAcceleration = 80;
    elevatorMotionMagicConfig.MotionMagicJerk = 400;

    elevatorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    elevator_FirstMotor.getConfigurator().apply(elevatorConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorConfig);
    elevator_FirstMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_SecondMotor.getConfigurator().apply(elevatorSlot0Config);
    elevator_FirstMotor.getConfigurator().apply(elevatorMotionMagicConfig);
    elevator_SecondMotor.getConfigurator().apply(elevatorMotionMagicConfig);

    ifChange_High = false;
    ifChange_Low = false;
  }

  public void intakeCoral() {
    goalPosition = ElevatorConstants.coralStationPosition; 
  }

  public void outCoral_L1() {goalPosition = ElevatorConstants.coralL1Position;}
  public void outCoral_L2() {goalPosition = ElevatorConstants.coralL2Position;}
  public void outCoral_L3() {goalPosition = ElevatorConstants.coralL3Position;}
  public void outCoral_L4() {goalPosition = ElevatorConstants.coralL4Position;}

  public void shootNet() {
    goalPosition = ElevatorConstants.algaeNetPosition;
  }

  public void shootProcessor() {
    goalPosition = ElevatorConstants.algaeProccesorPosition;
  }

  public void intakeAlgae_Low() {
    goalPosition = ElevatorConstants.algaeL2Position;
  }

  public void intakeAlgae_High() {
    goalPosition = ElevatorConstants.algaeL3Position;
  }

  public void intakeAlgae_Floor() {
    goalPosition = ElevatorConstants.algaeFloorPosition;
  }

  public void toPrimitive() {
    goalPosition = ElevatorConstants.primitivePosition;
  }

  public double getCurrentPosition() {
    return elevator_FirstMotor.getPosition().getValueAsDouble();
  }

  public boolean arriveSetPoint() {
    return (Math.abs(goalPosition - getCurrentPosition()) <= 2);
  }

  public boolean arrivePrimition() {
    return (Math.abs(ElevatorConstants.primitivePosition - getCurrentPosition()) <= 1);
  }

  public double getGoalPosition() {
    return goalPosition;
  }

  @Override
  public void periodic() {
    elevator_FirstMotor.setControl(request_Elevator.withPosition(goalPosition));

    if (getCurrentPosition() <= 30) {
      ElevatorConstants.arriveLow = true;
    }else{
      ElevatorConstants.arriveLow = false;
    }

    if ((getCurrentPosition() < 8 && lastPosition > 8)) {
      ifChange_Low = true;
    }else if((getCurrentPosition() > 8 && lastPosition < 8)){
      ifChange_High = true;
    }

    if(ifChange_Low) {
      elevatorSlot0Config.kS = 0;
      elevatorSlot0Config.kG = 0.35;
      elevatorSlot0Config.kV = 0;
      elevatorSlot0Config.kA = 0;
      elevatorSlot0Config.kP = 0.9;
      elevatorSlot0Config.kI = 0;
      elevatorSlot0Config.kD = 0;

      elevator_FirstMotor.getConfigurator().apply(elevatorSlot0Config);
      elevator_SecondMotor.getConfigurator().apply(elevatorSlot0Config);
      System.out.println("setlow");
      ifChange_Low = false;
    }else if(ifChange_High) {
      elevatorSlot0Config.kS = 0;
      elevatorSlot0Config.kG = 0.44;
      elevatorSlot0Config.kV = 0;
      elevatorSlot0Config.kA = 0;
      elevatorSlot0Config.kP = 0.9;
      elevatorSlot0Config.kI = 0;
      elevatorSlot0Config.kD = 0;

      elevator_FirstMotor.getConfigurator().apply(elevatorSlot0Config);
      elevator_SecondMotor.getConfigurator().apply(elevatorSlot0Config);

      ifChange_High = false;
      System.out.println("sethigh");
    }

    lastPosition = getCurrentPosition();

    //SmartDashboard
    SmartDashboard.putNumber("Intake/GoalPosition", goalPosition);
    SmartDashboard.putNumber("Intake/CurrentPosition", getCurrentPosition());
    SmartDashboard.putBoolean("Intake/ArrivePosition", arriveSetPoint());
  }
}
