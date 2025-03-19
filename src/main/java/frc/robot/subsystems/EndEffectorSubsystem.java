// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX wheelMotor;
  private final TalonFX pivotMotor;
  private final CANcoder pivotCANcoder;
  private final DigitalInput irSensor_CoralFirst;
  private final DigitalInput irSensor_CoralSecond;
  private final DigitalInput irSensor_Algae;

  private final TalonFXConfiguration wheelConfig;
  private final TalonFXConfiguration pivotConfig;
  private final CANcoderConfiguration CANcoderConfig;
  private final MotionMagicConfigs wheelMotionMagicConfig;
  private final MotionMagicConfigs turnMotionMagicConfigs;
  private final Slot0Configs wheelMotor_Slot0;
  private final Slot1Configs wheelMotor_Slot1;
  private final MotionMagicVelocityVoltage requst_wheelSpeed;
  

  private final PIDController armPID;
  private ArmFeedforward armFeedforward;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;
  private double goalAngle;

  private Debouncer m_Debouncer_first;
  private Debouncer m_Debouncer_second;

    public EndEffectorSubsystem() {
      // Motor controller
      wheelMotor = new TalonFX(EndEffectorConstants.intakeWheel_ID);
      pivotMotor = new TalonFX(EndEffectorConstants.intakeArm_ID);
      // Encoder
      pivotCANcoder = new CANcoder(EndEffectorConstants.armAbsolutedEncoder_ID);
      // IR sensor
      irSensor_CoralFirst = new DigitalInput(EndEffectorConstants.irSensor_CoralFirst_ID);
      irSensor_CoralSecond = new DigitalInput(EndEffectorConstants.irSensor_CoralSecond_ID);
      irSensor_Algae = new DigitalInput(EndEffectorConstants.irSensor_Algae_ID);
      // Init goal angle
      goalAngle = EndEffectorConstants.primitiveAngle;
      
  
      // Motor Configurations
      wheelConfig = new TalonFXConfiguration();
      pivotConfig = new TalonFXConfiguration();
      wheelMotionMagicConfig = new MotionMagicConfigs();
      turnMotionMagicConfigs = new MotionMagicConfigs();
      wheelMotor_Slot0 = wheelConfig.Slot0;
      wheelMotor_Slot1 = new Slot1Configs();
      requst_wheelSpeed = new MotionMagicVelocityVoltage(0);
  
      wheelConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      pivotConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      
      //slot0
      wheelMotor_Slot0.kS = 0;
      wheelMotor_Slot0.kV = 0;
      wheelMotor_Slot0.kA = 0;
      wheelMotor_Slot0.kP = 0.3;
      wheelMotor_Slot0.kI = 0;
      wheelMotor_Slot0.kD = 0;
  
      wheelMotor_Slot1.kG = 0;
      wheelMotor_Slot1.kS = 0;
      wheelMotor_Slot1.kV = 0;
      wheelMotor_Slot1.kA = 0;
      wheelMotor_Slot1.kP = 10;
      wheelMotor_Slot1.kI = 0;
      wheelMotor_Slot1.kD = 0;

      //MotioinMagic Config
      turnMotionMagicConfigs.MotionMagicCruiseVelocity = 40;
      turnMotionMagicConfigs.MotionMagicAcceleration = 80;
      turnMotionMagicConfigs.MotionMagicJerk = 400;

      wheelMotionMagicConfig.MotionMagicAcceleration = 400;
      wheelMotionMagicConfig.MotionMagicJerk = 4000;
      // Absolute Encoder Configurations
      CANcoderConfig = new CANcoderConfiguration();
      CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      CANcoderConfig.MagnetSensor.MagnetOffset = EndEffectorConstants.absolutedEncoderOffset;
      pivotCANcoder.getConfigurator().apply(CANcoderConfig);
  
      //Motor Configurations
      wheelMotor.getConfigurator().apply(wheelConfig);
      wheelMotor.getConfigurator().apply(wheelMotionMagicConfig);
      wheelMotor.getConfigurator().apply(wheelMotor_Slot0);
      pivotMotor.getConfigurator().apply(pivotConfig);
  
      // PID Controller and Feedforward
      armPID = new PIDController(EndEffectorConstants.armPID_Kp, EndEffectorConstants.armPID_Ki, EndEffectorConstants.armPID_Kd);
      armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);
      // armPID.setIntegratorRange(EndEffectorConstants.armPIDMinOutput, EndEffectorConstants.armPIDMaxOutput);

      // Init debouncer
      m_Debouncer_first = new Debouncer(0.05, DebounceType.kRising);
      m_Debouncer_second = new Debouncer(0.05, DebounceType.kRising);
    }
  
    // ======== Arm ========
    // Intake coral
    public void intakeCoral_Arm() {goalAngle = EndEffectorConstants.coralStationAngle;}
    // Shoot Reef
    public void Arm_IDLE() {goalAngle = EndEffectorConstants.primitiveAngle;}
    public void Arm_shootCoral_L1() {goalAngle = EndEffectorConstants.coralL1Angle;}
    public void Arm_shootCoral_L2() {goalAngle = EndEffectorConstants.coralL2Angle;}
    public void Arm_shootCoral_L3() {goalAngle = EndEffectorConstants.coralL3Angle;}
    public void Arm_shootCoral_L4() {goalAngle = EndEffectorConstants.coralL4Angle;}
    public void coralL4Primitive_Arm(){goalAngle = EndEffectorConstants.coralL4UpAngle;}

    public void Arm_NET_IDLE() {goalAngle = EndEffectorConstants.netUpAngle;}
    public void Arm_shootAlgae_NET() {goalAngle = EndEffectorConstants.algaeNetAngle;}
    public void Arm_shootAlgae_Processor() {goalAngle = EndEffectorConstants.algaeProccesorAngle;}
    // Intake algae
    public void Arm_intakeAlgae_Low() {goalAngle = EndEffectorConstants.algaeLowInAngle;}
    public void Arm_intakeAlgae_High() {goalAngle = EndEffectorConstants.algaeHighInAngle;}
    public void Arm_intakeAlgae_Floor() {goalAngle = EndEffectorConstants.algaeFloorAngle;}

    public void primitiveArm_HasCoral() {goalAngle = EndEffectorConstants.primitiveAngle_HasCoral;}
    public void Arm_RemoveAlgae() {goalAngle = EndEffectorConstants.algaeRemoveAngle;}

    // ======== Wheel ========
    // Inatake coral
    public void intakeCoral_Wheel() {wheelMotor.setControl(requst_wheelSpeed.withVelocity(EndEffectorConstants.coralInSpeed_RotionPerSecond));}
    public void intakeCoralSlow_Wheel() {wheelMotor.setControl(requst_wheelSpeed.withVelocity(EndEffectorConstants.coralInSpeedSlow_RotationPerSecond));}
    // Coral
    public void turnMore_Coral() {wheelMotor.setVoltage(EndEffectorConstants.coralTurnMore);}
    // Shoot coral
    public void Wheel_shootCoral_L1() {wheelMotor.setVoltage(EndEffectorConstants.coralL1OutVol);}
    public void Wheel_shootCoral_L2() {wheelMotor.setVoltage(EndEffectorConstants.coralL2OutVol);}
    public void Wheel_shootCoral_L3() {wheelMotor.setVoltage(EndEffectorConstants.coralL3OutVol);}
    public void Wheel_shootCoral_L4() {wheelMotor.setVoltage(EndEffectorConstants.coralL4OutVol);}
    // Intake Algae
    public void intakeAlgae_Low_Wheel() {wheelMotor.setVoltage(EndEffectorConstants.algaeLowInVol);}
    public void intakeAlgae_High_Wheel() {wheelMotor.setVoltage(EndEffectorConstants.algaeHighInVol);}
    public void intakeAlgae_Floor_Wheel() {wheelMotor.setVoltage(EndEffectorConstants.algaeFloorInVol);}
    // Shoot Algae
    public void Wheel_shootAlgae_NET() {wheelMotor.setVoltage(EndEffectorConstants.algaeShootNetVol);}
    public void Wheel_shootAlgae_Processor() {wheelMotor.setVoltage(EndEffectorConstants.algaeShootProcessorVol);}
    // Wheel control
    public void outAlgae() {wheelMotor.setVoltage(EndEffectorConstants.algaeOutVol);}
    public void holdAlgae() {wheelMotor.setVoltage(EndEffectorConstants.algaeHoldVol);}
    public void RemoveAlgae() {wheelMotor.setVoltage(EndEffectorConstants.algaeRemoveVol);}
    public void stopWheel() {wheelMotor.setControl(requst_wheelSpeed.withVelocity(0));}

    
    public double getPosition() {
      return pivotMotor.getPosition().getValueAsDouble();
    }
  
    public double getAbsolutePosition() {
      return pivotCANcoder.getAbsolutePosition().getValueAsDouble();
    }
  
    public double getAngle() {
      return pivotCANcoder.getAbsolutePosition().getValueAsDouble()*360;
    }
  
    public double getRadians() {
      return Units.degreesToRadians(getAngle());
    }
  
    public double getVelocity() {
      return Units.rotationsPerMinuteToRadiansPerSecond(pivotCANcoder.getVelocity().getValueAsDouble()*60);
    }
    
    // IR sensor
    public boolean getFirstIR() {
      return m_Debouncer_first.calculate(irSensor_CoralFirst.get());
    }
    public boolean getSecondIR() {
      return m_Debouncer_second.calculate(irSensor_CoralSecond.get());
    }

    public boolean getAlgaeIR() {
      return irSensor_Algae.get();
    }

    public boolean shouldCoralSlow() {
      return !getFirstIR() && !getSecondIR();
    }

    public boolean canUp() {
      return getFirstIR();
    }

    public boolean hasCoral() {
      return (getFirstIR()) && (!getSecondIR());
    }

    public boolean hasAlgae() {
      return !irSensor_Algae.get();
    }

    public boolean wheelOverCurrent(){
      return Math.abs(wheelMotor.getStatorCurrent().getValueAsDouble()) > 40;
    }
  
    public boolean arrivedSetpoint() {
      return (Math.abs(armPID.getError()) <= 2);
    }
  
  
    @Override
    public void periodic() {
      // Arm feedforward
      if(90 >= getAngle() && getAngle() > 80 || 75 >= getAngle() && getAngle() > 70) {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);
      }else if(80 >= getAngle() && getAngle() > 75) {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks4, EndEffectorConstants.armFeedforward_Kg4, EndEffectorConstants.armFeedforward_Kv4);
      }else if(70 >= getAngle() && getAngle() > 61.6){
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks2, EndEffectorConstants.armFeedforward_Kg2, EndEffectorConstants.armFeedforward_Kv2);
      }else {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks3, EndEffectorConstants.armFeedforward_Kg3, EndEffectorConstants.armFeedforward_Kv3);
      }
      
      // PID and Feedforward
      pidOutput = armPID.calculate(getAngle(), goalAngle);
      feedforwardOutput = armFeedforward.calculate(getRadians(), getVelocity())/12;
      pidOutput = Constants.setMaxOutput(pidOutput, EndEffectorConstants.armPIDMaxOutput);
      // Implement
      output = pidOutput + feedforwardOutput;
      pivotMotor.set(output);

      //Log
      SmartDashboard.putNumber("EndEffector/pidOutput", pidOutput);
      SmartDashboard.putNumber("EndEffector/feedforwardOutput", feedforwardOutput);
      SmartDashboard.putNumber("EndEffector/Output", output);
      SmartDashboard.putBoolean("EndEffector/arrivedSetpoint", arrivedSetpoint());
      SmartDashboard.putBoolean("EndEffector/FirstIR", getFirstIR());
      SmartDashboard.putBoolean("EndEffector/SecondIR", getSecondIR());
      SmartDashboard.putBoolean("EndEffector/AlgaeIR", getAlgaeIR());
      SmartDashboard.putBoolean("EndEffector/hasCoral", hasCoral());
      SmartDashboard.putBoolean("EndEffector/hasAlgae", hasAlgae());
      SmartDashboard.putNumber("EndEffector/PivotAbsPosition", getAbsolutePosition());
      SmartDashboard.putNumber("EndEffector/PivotAngle", getAngle());
      SmartDashboard.putNumber("EndEffector/PivotPosition", getPosition());
      SmartDashboard.putNumber("EndEffector/PivotGoal", goalAngle);
      SmartDashboard.putNumber("EndEffector/WheelCurrent", wheelMotor.getStatorCurrent().getValueAsDouble());
  }
}
