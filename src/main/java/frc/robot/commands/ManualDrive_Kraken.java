// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

public class ManualDrive_Kraken extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem_Kraken m_SwerveSubsystem_Kraken;

  private final DoubleSupplier xSpeedFunc;
  private final DoubleSupplier ySpeedFunc;
  private final DoubleSupplier zSpeedFunc;
  private final BooleanSupplier isSlowFunc;
  // private final BooleanSupplier needSlowFunc;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter zLimiter;

  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  private boolean isSlow;
  // private boolean needSlow;
  public ManualDrive_Kraken(SwerveSubsystem_Kraken swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier isSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem_Kraken = swerveSubsystem;
    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;
    this.isSlowFunc = isSlow;
    // this.needSlowFunc = needSlow;

    this.xLimiter = new SlewRateLimiter(5.5);
    this.yLimiter = new SlewRateLimiter(5.5);
    this.zLimiter = new SlewRateLimiter(5.5);

    addRequirements(m_SwerveSubsystem_Kraken);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.xSpeed = -xSpeedFunc.getAsDouble();
    this.ySpeed = -ySpeedFunc.getAsDouble();
    this.zSpeed = -zSpeedFunc.getAsDouble();

    this.xSpeed = MathUtil.applyDeadband(this.xSpeed, OperatorConstants.kJoystickDeadBand);
    this.ySpeed = MathUtil.applyDeadband(this.ySpeed, OperatorConstants.kJoystickDeadBand);
    this.zSpeed = MathUtil.applyDeadband(this.zSpeed, OperatorConstants.kJoystickDeadBand);

    this.xSpeed = xLimiter.calculate(this.xSpeed);
    this.ySpeed = yLimiter.calculate(this.ySpeed);
    this.zSpeed = zLimiter.calculate(this.zSpeed);

    this.isSlow = isSlowFunc.getAsBoolean();
    // this.needSlow = needSlowFunc.getAsBoolean();

    if(isSlow || ElevatorConstants.arriveLow == false) {
      xSpeed = xSpeed*Math.abs(xSpeed)*0.2;
      ySpeed = ySpeed*Math.abs(ySpeed)*0.2;
      zSpeed = zSpeed*Math.abs(zSpeed)*0.1;
    }else {
      xSpeed = xSpeed*Math.abs(xSpeed)*0.6;
      ySpeed = ySpeed*Math.abs(ySpeed)*0.6;
      zSpeed = zSpeed*Math.abs(zSpeed)*0.4;
    }

    SmartDashboard.putNumber("ManualDrive/Xspeed", xSpeed);
    SmartDashboard.putNumber("ManualDrive/Yspeed", ySpeed);
    SmartDashboard.putNumber("ManualDrive/Zspeed", zSpeed);

    m_SwerveSubsystem_Kraken.drive(this.xSpeed, this.ySpeed, this.zSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem_Kraken.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
