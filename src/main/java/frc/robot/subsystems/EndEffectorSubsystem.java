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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakewheel;
  private final TalonFX intakeArm;
  private final CANcoder armAbsolutedEncoder;
  private final DigitalInput irSensor_CoralFirst;
  private final DigitalInput irSensor_CoralSecond;
  private final DigitalInput irSensor_Algae;

  private final TalonFXConfiguration wheelConfig;
  private final TalonFXConfiguration armConfig;
  private final CANcoderConfiguration absolutedEncoderConfig;
  private final MotionMagicConfigs speedMotionMagicConfigs;
  private final MotionMagicConfigs turnMotionMagicConfigs;
  private final Slot0Configs wheelMotor_Slot0;
  private final Slot1Configs wheelMotor_Slot1;
  private final MotionMagicVelocityVoltage request_EndEffectorSpeed;
  

  private final PIDController armPID;
  private ArmFeedforward armFeedforward;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;
  private double arriveAngle;

  private Debouncer m_Debouncer;

    public EndEffectorSubsystem() {
      // Motor controller
      intakewheel = new TalonFX(EndEffectorConstants.intakeWheel_ID);
      intakeArm = new TalonFX(EndEffectorConstants.intakeArm_ID);
      // Encoder
      armAbsolutedEncoder = new CANcoder(EndEffectorConstants.armAbsolutedEncoder_ID);
      // IR sensor
      irSensor_CoralFirst = new DigitalInput(EndEffectorConstants.irSensor_CoralFirst_ID);
      irSensor_CoralSecond = new DigitalInput(EndEffectorConstants.irSensor_CoralSecond_ID);
      irSensor_Algae = new DigitalInput(EndEffectorConstants.irSensor_Algae_ID);
      arriveAngle = EndEffectorConstants.primitiveAngle;
      turnMotionMagicConfigs = new MotionMagicConfigs();
  
      // Motor Configurations
      wheelConfig = new TalonFXConfiguration();
      armConfig = new TalonFXConfiguration();
      speedMotionMagicConfigs = new MotionMagicConfigs();
      wheelMotor_Slot0 = wheelConfig.Slot0;
      wheelMotor_Slot1 = new Slot1Configs();
      request_EndEffectorSpeed = new MotionMagicVelocityVoltage(0);
  
      wheelConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      armConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      
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

      speedMotionMagicConfigs.MotionMagicAcceleration = 400;
      speedMotionMagicConfigs.MotionMagicJerk = 4000;
      // Absolute Encoder Configurations
      absolutedEncoderConfig = new CANcoderConfiguration();
      absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      absolutedEncoderConfig.MagnetSensor.MagnetOffset = EndEffectorConstants.absolutedEncoderOffset;
      armAbsolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);
  
      //Motor Configurations
      intakewheel.getConfigurator().apply(wheelConfig);
      intakewheel.getConfigurator().apply(speedMotionMagicConfigs);
      intakewheel.getConfigurator().apply(wheelMotor_Slot0);
      intakeArm.getConfigurator().apply(armConfig);
  
      // PID Controller and Feedforward
      armPID = new PIDController(EndEffectorConstants.armPID_Kp, EndEffectorConstants.armPID_Ki, EndEffectorConstants.armPID_Kd);
      armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);
      // armPID.setIntegratorRange(EndEffectorConstants.armPIDMinOutput, EndEffectorConstants.armPIDMaxOutput);

      // Init debouncer
      m_Debouncer = new Debouncer(0.2);
    }
  
    // ======== Arm ========
    // Intake coral
    public void intakeCoral_Arm() {arriveAngle = EndEffectorConstants.coralStationAngle;}
    // Shoot Reef
    public void Arm_IDLE() {arriveAngle = EndEffectorConstants.primitiveAngle;}
    public void Arm_shootCoral_L1() {arriveAngle = EndEffectorConstants.coralL1Angle;}
    public void Arm_shootCoral_L2() {arriveAngle = EndEffectorConstants.coralL2Angle;}
    public void Arm_shootCoral_L3() {arriveAngle = EndEffectorConstants.coralL3Angle;}
    public void Arm_shootCoral_L4() {arriveAngle = EndEffectorConstants.coralL4Angle;}
    public void coralL4Primitive_Arm(){arriveAngle = EndEffectorConstants.coralL4UpAngle;}

    public void Arm_NET_IDLE() {arriveAngle = EndEffectorConstants.netUpAngle;}
    public void Arm_shootAlgae_NET() {arriveAngle = EndEffectorConstants.algaeNetAngle;}
    public void Arm_shootAlgae_Processor() {arriveAngle = EndEffectorConstants.algaeProccesorAngle;}
    // Intake algae
    public void Arm_intakeAlgae_Low() {arriveAngle = EndEffectorConstants.algaeLowInAngle;}
    public void Arm_intakeAlgae_High() {arriveAngle = EndEffectorConstants.algaeHighInAngle;}
    public void Arm_intakeAlgae_Floor() {arriveAngle = EndEffectorConstants.algaeFloorAngle;}

    public void primitiveArm_HasCoral(){arriveAngle = EndEffectorConstants.primitiveAngle_HasCoral;}

    // ======== Wheel ========
    // Inatake coral
    public void intakeCoral_Wheel() {intakewheel.setControl(request_EndEffectorSpeed.withVelocity(EndEffectorConstants.coralInSpeed_RotionPerSecond));}
    public void intakeCoralSlow_Wheel() {intakewheel.setControl(request_EndEffectorSpeed.withVelocity(EndEffectorConstants.coralInSpeedSlow_RotationPerSecond));}
    // Coral
    public void turnMore_Coral() {intakewheel.setVoltage(EndEffectorConstants.coralTurnMore);}
    // Shoot coral
    public void Wheel_shootCoral_L1() {intakewheel.setVoltage(EndEffectorConstants.coralL1OutVol);}
    public void Wheel_shootCoral_L2() {intakewheel.setVoltage(EndEffectorConstants.coralL2OutVol);}
    public void Wheel_shootCoral_L3() {intakewheel.setVoltage(EndEffectorConstants.coralL3OutVol);}
    public void Wheel_shootCoral_L4() {intakewheel.setVoltage(EndEffectorConstants.coralL4OutVol);}
    // Intake Algae
    public void intakeAlgae_Low_Wheel() {intakewheel.setVoltage(EndEffectorConstants.algaeLowInVol);}
    public void intakeAlgae_High_Wheel() {intakewheel.setVoltage(EndEffectorConstants.algaeHighInVol);}
    public void intakeAlgae_Floor_Wheel() {intakewheel.setVoltage(EndEffectorConstants.algaeFloorInVol);}
    // Shoot Algae
    public void Wheel_shootAlgae_NET() {intakewheel.setVoltage(EndEffectorConstants.algaeShootNetVol);}
    public void Wheel_shootAlgae_Processor() {intakewheel.setVoltage(EndEffectorConstants.algaeShootProcessorVol);}
    // Wheel control
    public void outAlgae() {intakewheel.setVoltage(EndEffectorConstants.algaeOutVol);}
    public void holdAlgae(){intakewheel.setVoltage(EndEffectorConstants.algaeHoldVol);}
    public void stopWheel() {intakewheel.setControl(request_EndEffectorSpeed.withVelocity(0));}
  
    
    public double getPosition() {
      return intakeArm.getPosition().getValueAsDouble();
    }
  
    public double getAbsolutePosition() {
      return armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble();
    }
  
    public double getAngle() {
      return armAbsolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }
  
    public double getRadians() {
      return Units.degreesToRadians(getAngle());
    }
  
    public double getVelocity() {
      return Units.rotationsPerMinuteToRadiansPerSecond(armAbsolutedEncoder.getVelocity().getValueAsDouble()*60);
    }
    
    // IR sensor
    public Boolean getFirstIR() {
      return irSensor_CoralFirst.get();
    }
    public Boolean getSecondIR() {
      return (irSensor_CoralSecond.get());
    }

    public boolean shouldCoralSlow() {return !getFirstIR() && !getSecondIR();}

    public boolean canUp() {return getFirstIR();}

    public boolean hasCoral() {return (getFirstIR()) && (!getSecondIR());}


    // public boolean shouldCoralSlow() {return !irSensor_CoralSecond.get() && !irSensor_CoralFirst.get();}

    // public boolean canUp() {return irSensor_CoralFirst.get();}

    // public boolean hasCoral() {return (irSensor_CoralFirst.get()) && (!irSensor_CoralSecond.get());}
  
    public boolean hasAlgae() {
      return wheelOverCurrent();
    }

    public boolean wheelOverCurrent(){
      return Math.abs(intakewheel.getStatorCurrent().getValueAsDouble()) > 40;
    }
  
    public boolean arrivedSetpoint() {
      return (Math.abs(armPID.getError()) <= 2);
    }
  
  
    @Override
    public void periodic() {
      // Arm
      if(90 >= getAngle() && getAngle() > 80 || 75 >= getAngle() && getAngle() > 70) {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks, EndEffectorConstants.armFeedforward_Kg, EndEffectorConstants.armFeedforward_Kv);
      }else if(80 >= getAngle() && getAngle() > 75) {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks4, EndEffectorConstants.armFeedforward_Kg4, EndEffectorConstants.armFeedforward_Kv4);
      }else if(70 >= getAngle() && getAngle() > 61.6){
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks2, EndEffectorConstants.armFeedforward_Kg2, EndEffectorConstants.armFeedforward_Kv2);
      }else {
        armFeedforward = new ArmFeedforward(EndEffectorConstants.armFeedforward_Ks3, EndEffectorConstants.armFeedforward_Kg3, EndEffectorConstants.armFeedforward_Kv3);
      }
    pidOutput = armPID.calculate(getAngle(), arriveAngle);
    feedforwardOutput = armFeedforward.calculate(getRadians(), getVelocity())/12;
    pidOutput = Constants.setMaxOutput(pidOutput, EndEffectorConstants.armPIDMaxOutput);
  
    
    output = pidOutput + feedforwardOutput;
    intakeArm.set(output);

    //SmartDashboard
    SmartDashboard.putNumber("EndEffector/pidOutput", pidOutput);
    SmartDashboard.putNumber("EndEffector/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("EndEffector/Output", output);
    SmartDashboard.putBoolean("EndEffector/arrivedSetpoint", arrivedSetpoint());
    SmartDashboard.putBoolean("EndEffector/First IR", getFirstIR());
    SmartDashboard.putBoolean("EndEffector/Second IR", getSecondIR());
    SmartDashboard.putBoolean("EndEffector/hasCoral", hasCoral());
    SmartDashboard.putBoolean("EndEffector/HasSensorAlgae", hasAlgae());
    SmartDashboard.putNumber("EndEffector/AbsolutedArmPosition", getAbsolutePosition());
    SmartDashboard.putNumber("EndEffector/ArmAngle", getAngle());
    SmartDashboard.putNumber("EndEffector/getPosition", getPosition());
    SmartDashboard.putNumber("EndEffector Current", intakewheel.getStatorCurrent().getValueAsDouble());
  }
}
