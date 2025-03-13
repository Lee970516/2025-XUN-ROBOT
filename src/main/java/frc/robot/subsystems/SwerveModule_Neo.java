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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Module_NeoConstants;

public class SwerveModule_Neo extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SparkMax turningMotor;
  private final SparkMax driveMotor;

  private final RelativeEncoder driveMotorEncoder;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration cancoderConfig;

  private final SparkMaxConfig turningMotorConfig;
  private final SparkMaxConfig driveMotorConfig;

  private final PIDController turningPidController;
  private final SimpleMotorFeedforward driveFeedForward;

  public SwerveModule_Neo(int turningMotor_ID, int driveMotor_ID, int absolutedEncoder_ID, double offset) {
    turningMotor = new SparkMax(turningMotor_ID, MotorType.kBrushless);
    driveMotor = new SparkMax(driveMotor_ID, MotorType.kBrushless);

    driveMotorEncoder = driveMotor.getEncoder();

    turningMotorConfig = new SparkMaxConfig();
    driveMotorConfig = new SparkMaxConfig();

    absolutedEncoder = new CANcoder(absolutedEncoder_ID);
    cancoderConfig = new CANcoderConfiguration();

    turningPidController = new PIDController(Module_NeoConstants.turningPidController_Kp, Module_NeoConstants.turningPidController_Ki, Module_NeoConstants.turningPidController_Kd);
    turningPidController.enableContinuousInput(Module_NeoConstants.pidRangeMin, Module_NeoConstants.pidRangeMax);

    driveFeedForward = new SimpleMotorFeedforward(Module_NeoConstants.driveFeedforward_Ks, Module_NeoConstants.driveFeedforward_Kv);

    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = offset;

    turningMotorConfig.inverted(true);
    driveMotorConfig.inverted(true);

    turningMotorConfig.idleMode(IdleMode.kBrake);
    driveMotorConfig.idleMode(IdleMode.kBrake);

    turningMotor.configure(turningMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    driveMotor.configure(driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    absolutedEncoder.getConfigurator().apply(cancoderConfig);

    resetEncoder();
  }

  public void resetEncoder() {
    driveMotorEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity()*Module_NeoConstants.driveEncoderRot2Meter;
  }

  public double getDrivePosition() {
    return driveMotorEncoder.getPosition()*Module_NeoConstants.driveEncoderRot2Meter;
  }

  public double getTurningPosition() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getTurningMotorPosition(){
    return turningMotor.getEncoder().getPosition();
  }

  public double getTurningAngle() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public void stopMotor() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void setState(SwerveModuleState state) {
    // Turn Motor
      state.optimize(getState().angle);
      double turningMotorOutput = turningPidController.calculate(getState().angle.getDegrees(), state.angle.getDegrees());
      turningMotor.set(turningMotorOutput);
    // Drive motor
      double driveMotorOutput = driveFeedForward.calculate(state.speedMetersPerSecond)/12;
      driveMotor.set(driveMotorOutput);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
