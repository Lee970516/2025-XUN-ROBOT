// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final SparkMax firstMotor;
  private final SparkMax secondMotor;

  private final SparkMaxConfig firstMotorConfig;
  private final SparkMaxConfig secondMotorConfig;

  private final RelativeEncoder firstEncoder;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration absolutedEncoderConfig;

  private final PIDController climbPID;

  private double pidOutput;
  private double goalAngle;

  public ClimberSubsystem() {
    goalAngle = ClimberConstants.climbInAngle;
    //Climber Motor
    firstMotor = new SparkMax(ClimberConstants.firstMotor_ID, MotorType.kBrushless);
    secondMotor = new SparkMax(ClimberConstants.secondMotor_ID, MotorType.kBrushless);

    firstMotorConfig = new SparkMaxConfig();
    secondMotorConfig = new SparkMaxConfig();
    
    firstMotorConfig.idleMode(IdleMode.kBrake);
    secondMotorConfig.idleMode(IdleMode.kBrake);
    firstMotorConfig.inverted(ClimberConstants.firstMotorReverse);
    secondMotorConfig.inverted(ClimberConstants.secondMotorReverse);

    secondMotorConfig.follow(firstMotor.getDeviceId(), true);

    firstMotor.configure(firstMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    secondMotor.configure(secondMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //Climber AbsolutedEncoder
    absolutedEncoder = new CANcoder(ClimberConstants.absolutedEncoder_ID);
    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = ClimberConstants.absolutedEncoderOffset;

    firstEncoder = firstMotor.getEncoder();

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

  public double getRelativePosition() {
    return firstEncoder.getPosition();
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
    firstMotor.set(pidOutput);

    SmartDashboard.putNumber("Climber/AbsolutedPosition", getAbsolutedPosition());
    SmartDashboard.putNumber("Climber/GoalAngle", goalAngle);
    SmartDashboard.putNumber("Climber/CurrentAngle", getAngle());
    SmartDashboard.putNumber("Climber/RelativePositon", getRelativePosition());
  }
}
