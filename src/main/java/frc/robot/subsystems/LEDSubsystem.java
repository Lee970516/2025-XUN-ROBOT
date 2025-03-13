// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle candle;
  private final CANdleConfiguration candleConfig;
  private final int ledNum;
  private Animation ledAnimation;

  private Timer timer;

  public LEDSubsystem() {
    candle = new CANdle(LEDConstants.candle_ID);
    candleConfig = new CANdleConfiguration();

    candleConfig.stripType = LEDStripType.RGB;
    candleConfig.statusLedOffWhenActive = true;
    candleConfig.disableWhenLOS = false;
    candleConfig.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfig);

    ledNum = LEDConstants.ledNum;

    ledAnimation = null;
    timer = new Timer();

    normal();
  }

  public void fireAnimation() {
    ledAnimation = new FireAnimation(0.6, 0.2, ledNum, 1, 0.2, false, 0);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void hasGamePiece() {
    ledAnimation = new StrobeAnimation(0, 127, 0);
    candle.animate(ledAnimation);
    timer.reset();
    timer.start();
    LEDConstants.LEDFlag = false;
  }

  public void tracking() {
    ledAnimation = new StrobeAnimation(0, 0, 127);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void arrivePosition_Base() {
    candle.animate(null);
    candle.setLEDs(0, 0, 127, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }
  
  public void canTrackLeft() {
    ledAnimation = new StrobeAnimation(127, 0, 127);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void canTrackRight() {
    ledAnimation = new StrobeAnimation(127, 165, 0);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void noTarget() {
    ledAnimation = new FireAnimation(0.01, 0.2, ledNum, 1, 0);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void intakeGamePiece() {
    ledAnimation = new StrobeAnimation(127, 0, 0);
    candle.animate(ledAnimation);
    // candle.setLEDs(127, 0, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void arrivePosition_Intake() {
    // ledAnimation = new RainbowAnimation(0.2, 0.5, ledNum, false, 9);
    candle.animate(null);
    LEDConstants.LEDFlag = false;
  }

  public void intakeArriving() {
    ledAnimation = new FireAnimation(0.6, 0.2, ledNum, 1, 0.2, false, 9);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void shootGamePiece() {
    ledAnimation = new StrobeAnimation(77, 0, 9);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void climbing() {
    ledAnimation = new StrobeAnimation(ledNum, ledNum, ledNum);
    candle.animate(ledAnimation);
    candle.setLEDs(0, 0, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void onCage() {
    candle.animate(null);
    candle.setLEDs(0, 0, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }


  public void normal() {
    candle.animate(null);
    candle.setLEDs(0, 0, 0);
    LEDConstants.LEDFlag = false;
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(LEDConstants.LEDFlag) {
      if(LEDConstants.fireAnimation) fireAnimation();
      else if(LEDConstants.shootGamePiece) shootGamePiece();
      else if(LEDConstants.arrivePosition_Base) arrivePosition_Base();
      else if(LEDConstants.tracking) tracking();
      else if(LEDConstants.canTrackRight) canTrackLeft();
      else if(LEDConstants.canTrackLeft) canTrackRight();
      else if(LEDConstants.arrivePosition_Intake) arrivePosition_Intake();
      else if(LEDConstants.intakeArriving) intakeArriving();
      else if(LEDConstants.hasGamePiece) hasGamePiece();
      else if(LEDConstants.intakeGamePiece) intakeGamePiece();
      else if(LEDConstants.normal) normal();
    }
    // candle.animate(null);
    // candle.setLEDs(0, 0, 0);
    SmartDashboard.putBoolean("LED/BaseArriveSetpoint", LEDConstants.arrivePosition_Base);
    SmartDashboard.putBoolean("LED/ElevatorArriveSetpoint", LEDConstants.arrivePosition_Intake);
  }
}
