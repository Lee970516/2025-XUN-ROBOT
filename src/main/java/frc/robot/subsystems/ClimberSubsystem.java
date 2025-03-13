// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final SparkMax climbMotor;
  private final SparkMaxConfig climbConfig;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration absolutedEncoderConfig;

  private final PIDController climbPID;

  private double pidOutput;
  private double goalAngle;

  public ClimberSubsystem() {
    goalAngle = ClimberConstants.climbInAngle;
    //Climber Motor
    climbMotor = new SparkMax(ClimberConstants.climbMotor_ID, MotorType.kBrushless);
    climbConfig = new SparkMaxConfig();

    climbConfig.idleMode(IdleMode.kBrake);
    climbConfig.inverted(ClimberConstants.MotorReverse);

    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //Climber AbsolutedEncoder
    absolutedEncoder = new CANcoder(ClimberConstants.absolutedEncoder_ID);
    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = ClimberConstants.absolutedEncoderOffset;

    absolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);

    // PID and Feedforward
    climbPID = new PIDController(ClimberConstants.climbPID_Kp, ClimberConstants.climbPID_Ki, ClimberConstants.climbPID_Kd);
    // climbPID.setIntegratorRange(ClimberConstants.climbPIDMinOutput, ClimberConstants.climbPIDMaxOutput);
  }

  public double getAngle() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getAbsolutedPosition() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public void setOutAngle(){
    goalAngle = ClimberConstants.climbOutAngle;
  }

  public void setInAngle(){
    goalAngle = ClimberConstants.climbInAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidOutput = climbPID.calculate(getAngle(), goalAngle);
    climbMotor.set(pidOutput);
  }
}
