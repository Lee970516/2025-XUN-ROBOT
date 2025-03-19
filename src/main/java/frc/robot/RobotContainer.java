// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ManualDrive_Kraken;
import frc.robot.commands.AutoCommand.Coral_L1_Elevator_Auto;
import frc.robot.commands.AutoCommand.Coral_L2_Elevator_Auto;
import frc.robot.commands.AutoCommand.Coral_L3_Elevator_Auto;
import frc.robot.commands.AutoCommand.Coral_L4_Elevator_Auto;
import frc.robot.commands.AutoCommand.PrimitiveIntake_Auto;
import frc.robot.commands.AutoCommand.ShootCoral_Auto;
import frc.robot.commands.AutoCommand.TrackLeftReef_Auto;
import frc.robot.commands.AutoCommand.TrackRightReef_Auto;
import frc.robot.commands.ManualCommands.Coral_L1;
import frc.robot.commands.ManualCommands.Coral_L2;
import frc.robot.commands.ManualCommands.Coral_L3;
import frc.robot.commands.ManualCommands.Coral_L4;
import frc.robot.commands.ManualCommands.IntakeAlgae_Floor;
import frc.robot.commands.ManualCommands.IntakeAlgae_High;
import frc.robot.commands.ManualCommands.IntakeAlgae_Low;
import frc.robot.commands.ManualCommands.IntakeCoral;
import frc.robot.commands.ManualCommands.OutAlgae;
import frc.robot.commands.ManualCommands.PrimitiveIntake;
import frc.robot.commands.ManualCommands.RemoveAlgae;
import frc.robot.commands.ManualCommands.ShootNet;
import frc.robot.commands.ManualCommands.ShootProcessor;
import frc.robot.commands.ManualCommands.TurnMore;
import frc.robot.commands.TrackCommands.TrackLeftReef;
import frc.robot.commands.TrackCommands.TrackRightReef;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem = new PhotonVisionSubsystem();
  private final SwerveSubsystem_Kraken m_SwerveSubsystem = new SwerveSubsystem_Kraken();
  // private final SwerveSubsystem_Neo m_SwerveSubsystem_Neo = new SwerveSubsystem_Neo(m_PhotonVisionSubsystem);
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final EndEffectorSubsystem m_EndEffectorSubsystem = new EndEffectorSubsystem();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(); 

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Configure the trigger bindings

    // NamedCommands.registerCommand("Coral_L3_Left", new Coral_L3_Auto_LeftReef(m_ElevatorSubsystem, m_EndEffectorSubsystem, m_SwerveSubsystem, m_PhotonVisionSubsystem));
    // NamedCommands.registerCommand("Coral_L3_Right", new Coral_L3_Auto_RightReef(m_ElevatorSubsystem, m_EndEffectorSubsystem, m_SwerveSubsystem, m_PhotonVisionSubsystem));
    // NamedCommands.registerCommand("TrackLeftReef", new TrackLeftReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(5));
    // NamedCommands.registerCommand("Coral_L4_RightReef", new Coral_L4_Auto_RightReef(m_ElevatorSubsystem, m_EndEffectorSubsystem, m_SwerveSubsystem, m_PhotonVisionSubsystem));


    NamedCommands.registerCommand("stopMotor", Commands.runOnce(() -> m_SwerveSubsystem.stopMotor(), m_SwerveSubsystem));
    NamedCommands.registerCommand("IntakeCoral_Reef", new IntakeCoral(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.3));
    NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(5));
    NamedCommands.registerCommand("PrimitiveIntake", new PrimitiveIntake_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.4));
    NamedCommands.registerCommand("Coral_L4_Intake_WithTrack", new Coral_L4_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("Coral_L4_Intake", new Coral_L4_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.4));
    NamedCommands.registerCommand("Coral_L2_Intake", new Coral_L2_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.2));
    NamedCommands.registerCommand("Coral_L3_Intake", new Coral_L3_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.3));
    NamedCommands.registerCommand("Coral_L1_Intake", new Coral_L1_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.1));
    NamedCommands.registerCommand("IntakeCoral_IDLE", new IntakeCoral(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.8));
    NamedCommands.registerCommand("TrackLeftReef_Auto", new TrackLeftReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("TrackRightReef_Auto", new TrackRightReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("TrackRightReef_Auto_LeftSide", new TrackRightReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(0.6));
    NamedCommands.registerCommand("ShootCoral_Auto", new ShootCoral_Auto(m_EndEffectorSubsystem).withTimeout(0.3));

    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Driver Controller
    DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
    DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
    DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);

    BooleanSupplier isSlowFunc = ()-> driverController.getHID().getRightTriggerAxis() > 0.2;
    BooleanSupplier ifFeed = ()-> driverController.getHID().getLeftTriggerAxis() > 0.2;

    driverController.leftBumper().whileTrue(new TrackLeftReef(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    // driverController.leftTrigger(0.4).toggleOnTrue(new TrackMiddleReef(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    driverController.rightBumper().whileTrue(new TrackRightReef(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    // driverController.a().toggleOnTrue(new TrackCage(m_SwerveSubsystem, m_PhotonVisionSubsystem));

    driverController.y().whileTrue(
      Commands.runOnce(()->{
        m_SwerveSubsystem.resetGyro();
      })
    );

    // Operator Controller
    operatorController.pov(180).toggleOnTrue(new Coral_L1(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.pov(0).toggleOnTrue(new Coral_L2(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.leftTrigger().toggleOnTrue(new Coral_L3(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.leftBumper().toggleOnTrue(new Coral_L4(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.pov(270).toggleOnTrue(new ShootProcessor(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.pov(90).toggleOnTrue(new IntakeAlgae_Floor(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.rightTrigger().toggleOnTrue(new IntakeAlgae_Low(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.rightBumper().toggleOnTrue(new IntakeAlgae_High(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    
    operatorController.x().toggleOnTrue(new RemoveAlgae(m_EndEffectorSubsystem, m_ElevatorSubsystem));
    operatorController.a().toggleOnTrue(new PrimitiveIntake(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.b().toggleOnTrue(new IntakeCoral(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.y().toggleOnTrue(new ShootNet(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.axisGreaterThan(5, 0.6).whileTrue(new OutAlgae(m_EndEffectorSubsystem));
    operatorController.axisGreaterThan(1, 0.6).whileTrue(new TurnMore(m_EndEffectorSubsystem));

    // Set the default drive command 
    m_SwerveSubsystem.setDefaultCommand(new ManualDrive_Kraken(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc, isSlowFunc));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
} 
