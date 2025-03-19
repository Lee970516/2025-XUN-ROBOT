// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static double setMaxOutput(double output, double maxOutput){
    return Math.min(maxOutput, Math.max(-maxOutput, output));
  }

  // public static Double[] optimate(double currentAngle, double goalAngle, double speedMetersPerSecond){
  //   Double[] goal = new Double[2];
  //   double delta = Math.abs(goalAngle - currentAngle);
  //   if (delta > (Math.PI / 2)) {
  //     goalAngle = goalAngle - Math.PI;
  //     speedMetersPerSecond = speedMetersPerSecond * -1;
  //   }
  //   goal[0] = goalAngle;
  //   goal[1] = speedMetersPerSecond;
  //   return goal;
  // }

  public static class Module_KrakenConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2RadPerSec = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;
    public static final double driveEncoderRot2RadPerMin = turningEncoderRot2RadPerSec*60;

    public static final double kModuleDistance = 22.24*0.0254;

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0.00001;

    public static final double drivePidController_Kp = 0;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.13;
    public static final double driveFeedforward_Kv = 2;
    public static final double driveFeedforward_Ka = 0;

  }

  public class Swerve_KrakenConstants {
    public static final int leftFrontDrive_ID = 2;
    public static final int leftBackDrive_ID = 1;
    public static final int rightFrontDrive_ID = 3;
    public static final int rightBackDrive_ID = 4;

    public static final int leftFrontTurning_ID = 15;
    public static final int leftBackTurning_ID = 5;
    public static final int rightFrontTurning_ID = 16;
    public static final int rightBackTurning_ID = 17;

    public static final int leftFrontAbsolutedEncoder_ID = 42;
    public static final int leftBackAbsolutedEncoder_ID = 41;
    public static final int rightFrontAbsolutedEncoder_ID = 43;


    public static final int rightBackAbsolutedEncoder_ID = 44;

    public static final double leftFrontOffset = -0.352806;
    public static final double leftBackOffset = 0.138427;
    public static final double rightFrontOffset = -0.280517;
    public static final double rightBackOffset = 0.161621;
    public static final int gyro_ID = 56;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;

    public static final double kModuleDistance = 22.24*0.0254;


    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2), 
      new Translation2d(kModuleDistance/2, -kModuleDistance/2), 
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double pathingMoving_Kp = 5;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;

    public static final double pathingtheta_Kp = 0.6;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 5.94;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 850;
  
  }

  public class PhotonConstants {
    public static final double xPidController_Kp = 0.6;
    public static final double xPidController_Ki = 0;
    public static final double xPidController_Kd = 0;

    public static final double yPidController_Kp = 0.6;
    public static final double yPidController_Ki = 0;
    public static final double yPidController_Kd = 0.0015;

    public static final double rotationPidController_Kp = 0.0015;
    public static final double rotationPidController_Ki = 0;
    public static final double rotationPidController_Kd = 0.000;

    public static final double xPidMaxOutput = 0.4;
    public static final double yPidMaxOutput = 0.4;
    public static final double rotationPidMaxOutput = 0.4;
    public static final double xPidMaxOutput_NeedSlow = 0.2;
    public static final double yPidMaxOutput_NeedSlow = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow = 0.2;

    public static final double xPidMaxOutput_Reef = 0.3;
    public static final double yPidMaxOutput_Reef = 0.3;
    public static final double rotationPidMaxOutput_Reef = 0.3;
    public static final double xPidMaxOutput_NeedSlow_Reef= 0.2;
    public static final double yPidMaxOutput_NeedSlow_Reef = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Reef = 0.1;

    public static final double xPidMaxOutput_CoralStation = 0.4;
    public static final double yPidMaxOutput_CoralStation = 0.4;
    public static final double rotationPidMaxOutput_CoralStation = 0.4;
    public static final double xPidMaxOutput_NeedSlow_CoralStation = 0.2;
    public static final double yPidMaxOutput_NeedSlow_CoralStation = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_CoralStation = 0.2;

    public static final double xPidMaxOutput_Cage = 0.4;
    public static final double yPidMaxOutput_Cage = 0.4;
    public static final double rotationPidMaxOutput_Cage = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Cage = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Cage = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Cage = 0.2;

    public static final double xPidMaxOutput_Net = 0.4;
    public static final double yPidMaxOutput_Net = 0.4;
    public static final double rotationPidMaxOutput_Net = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Net = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Net = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Net = 0.2;

    public static final double xPidMaxOutput_Processor = 0.4;
    public static final double yPidMaxOutput_Processor = 0.4;
    public static final double rotationPidMaxOutput_Processor = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Processor = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Processor = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Processor = 0.2;

    public static final double xPidSetPoint_RightReef = 0.428; 
    public static final double yPidSetPoint_RightReef = -0.127;
    public static final double rotationPidSetPoint_RightReef = 181;

    public static final double xPidSetPoint_LeftReef = 0.445;
    public static final double yPidSetPoint_LeftReef = 0.125;
    public static final double rotationPidSetPoint_LeftReef = 181.2;

    public static final double xPidSetPoint_MiddleReef_FrontRight = 0;
    public static final double yPidSetPoint_MiddleReef_FrontRight = 0;
    public static final double rotationPidSetPoint_MiddleReef_FrontRight = 182;

    public static final double xPidSetPoint_MiddleReef_FrontLeft = 0;
    public static final double yPidSetPoint_MiddleReef_FrontLeft = 0;
    public static final double rotationPidSetPoint_MiddleReef_FrontLeft = 0;

    public static final double xPidSetPoint_CoralStation_Back = 0;
    public static final double yPidSetPoint_CoralStation_Back = 0;
    public static final double rotationPidSetPoint_CoralStation_Back = 0;

    public static final double xPidSetPoint_CoralStation_FrontRight = 0;
    public static final double yPidSetPoint_CoralStation_FrontRight = 0;
    public static final double rotationPidSetPoint_CoralStation_FrontRight = 0;

    public static final double xPidSetPoint_Cage_FrontRight = 0;
    public static final double yPidSetPoint_Cage_FrontRight = 0;
    public static final double rotationPidSetPoint_Cage_FrontRight = 0;

    public static final double xPidSetPoint_Cage_FrontLeft = 0;
    public static final double yPidSetPoint_Cage_FrontLeft = 0;
    public static final double rotationPidSetPoint_Cage_FrontLeft = 0;

    public static final double xPidSetPoint_Cage_Back_ID20_ID11 = 0;
    public static final double yPidSetPoint_Cage_Back_ID20_ID11 = 0;
    public static final double rotationPidSetPoint_Cage_Back_ID20_ID11 = 0;

    public static final double xPidSetPoint_Cage_Back_ID21_ID10 = 0;
    public static final double yPidSetPoint_Cage_Back_ID21_ID10 = 0;
    public static final double rotationPidSetPoint_Cage_Back_ID21_ID10 = 0;

    public static final double xPidSetPoint_Processor_FrontRight = 0;
    public static final double yPidSetPoint_Processor_FrontRight = 0;
    public static final double rotationPidSetPoint_Processor_FrontRight = 0;

    public static final double xPidSetPoint_Processor_FrontLeft = 0;
    public static final double yPidSetPoint_Processor_FrontLeft = 0;
    public static final double rotationPidSetPoint_Processor_FrontLeft = 0;

    public static final double xPidSetPoint_Net_FrontRight = 0;
    public static final double yPidSetPoint_Net_FrontRight = 0;
    public static final double rotationPidSetPoint_Net_FrontRight = 0;

    public static final double xPidSetPoint_Net_FrontLeft = 0;
    public static final double yPidSetPoint_Net_FrontLeft = 0;
    public static final double rotationPidSetPoint_Net_FrontLeft = 0;

    public static final double xPidSetPoint_Net_Back_ID20_ID11 = 0;
    public static final double yPidSetPoint_Net_Back_ID20_ID11 = 0;
    public static final double rotationPidSetPoint_Net_Back_ID20_ID11 = 0;

    public static final double xPidSetPoint_Net_Back_ID21_ID10 = 0;
    public static final double yPidSetPoint_Net_Back_ID21_ID10 = 0;
    public static final double rotationPidSetPoint_Net_Back_ID21_ID10 = 0;

    public static final double arriveXPosition_Reef = 0;
    public static final double arriveXPosition_Cage = 0;
    public static final double arrivePosition_Net = 0;

    public static final double tooClosePosition_Reef = 0;
    public static final double tooClosePosition_Cage = 0;
    public static final double tooClosePosition_Net = 0;

  }

  public class ElevatorConstants {
    public static final int elevator_FirstMotor_ID = 6;
    public static final int elevator_SecondMotor_ID = 7;

    public static final double primitivePosition = -0.2;//-0.14
    public static final double coralL1Position = 4;//3.72
    public static final double coralL2Position = 11.44;//7.42
    public static final double coralL3Position = 21.54;//21.42
    public static final double coralL4Position = 43.34;//42.52
    public static final double coralStationPosition = -0.2;//0.12

    public static final double algaeFloorPosition = -0.2;//0.12
    public static final double algaeNetPosition = 45.7;//0.12
    public static final double algaeL2Position = 2.84;//3.82
    public static final double algaeL3Position = 16;//15.82

    public static final double algaeProccesorPosition = -0.2;//0.12

    public static boolean arriveLow = false;
  }

  public static class Mode{
    public static BooleanSupplier changeModeFunc = () -> nowModeIsCoral();
    public static String nowMode = "coralMode";

    public static void changeMode(){
      if (nowMode == "coralMode") {
        nowMode = "algaeMode";
      }else{
        nowMode = "coralMode";
      }
    }

    public static boolean nowModeIsCoral(){
      return nowMode == "coralMode";
    }

    public static boolean nowModeIsAlgae() {
      return nowMode == "algaeMode";
    }
  }

  public static class EndEffectorConstants {
    public static final int intakeWheel_ID = 13;
    public static final int intakeArm_ID = 14;
    public static final int armAbsolutedEncoder_ID = 45;
    public static final int irSensor_CoralFirst_ID = 0;
    public static final int irSensor_CoralSecond_ID = 1;
    public static final int irSensor_Algae_ID = 2;

    public static final double absolutedEncoderOffset = 0.140625;

    public static final double armPID_Kp = 0.0051;//0.0048  
    public static final double armPID_Ki = 0;
    public static final double armPID_Kd = 0.0001;//0.0001
    public static final double armPIDMinOutput = 0;
    public static final double armPIDMaxOutput = 0.2;

    public static final double armFeedforward_Ks = 0;
    public static final double armFeedforward_Kg = 0.6;//
    public static final double armFeedforward_Kv = 0;
    

    public static final double armFeedforward_Ks2 = 0;
    public static final double armFeedforward_Kg2 = 0.4;//0.4
    public static final double armFeedforward_Kv2 = 0;

    public static final double armFeedforward_Ks3 = 0;
    public static final double armFeedforward_Kg3 = 0.3;//0.3
    public static final double armFeedforward_Kv3 = 0;

    public static final double armFeedforward_Ks4 = 0;
    public static final double armFeedforward_Kg4 = 0.6;//0.6
    public static final double armFeedforward_Kv4 = 0;

    public static final double primitiveAngle = 82;
    public static final double primitiveAngle_HasCoral = 82;
    public static final double coralL1Angle = 82;
    public static final double coralL2Angle = 75;
    public static final double coralL3Angle = 82;
    public static final double coralL4Angle = 56;//not yet
    public static final double coralStationAngle = 76;
    public static final double coralL4UpAngle = 70;
    public static final double algaeFloorAngle = 7;
    public static final double algaeNetAngle = 100;//not yet
    public static final double netUpAngle = 82;
    public static final double algaeLowInAngle = 64;
    public static final double algaeHighInAngle = 64;
    public static final double algaeProccesorAngle = 9;//not yet
    public static final double algaeRemoveAngle = 75;
    

    public static final double coralL1OutVol = -3.5;
    public static final double coralL2OutVol = -2;
    public static final double coralL3OutVol = -2;
    public static final double coralL4OutVol = -2;
    public static final double coralTurnMore = -0.5;
    public static final double coralInSpeed_RotionPerSecond = -32;
    public static final double coralInSpeedSlow_RotationPerSecond = -5;
    public static final double algaeFloorInVol = -6;
    public static final double algaeShootNetVol = 6;
    public static final double algaeLowInVol = -6;
    public static final double algaeHighInVol = -6;
    public static final double algaeShootProcessorVol = 3;
    public static final double algaeHoldVol = -4;  
    public static final double algaeOutVol = 6; 
    public static final double algaeRemoveVol = -6;
  }

  public static class ClimberConstants {
    public static final int firstMotor_ID = 0;
    public static final int secondMotor_ID = 0;

    public static final int absolutedEncoder_ID = 0;

    public static final boolean firstMotorReverse = false;
    public static final boolean secondMotorReverse = false;

    public static final double absolutedEncoderOffset = 0;

    public static final double climbPID_Kp = 0;
    public static final double climbPID_Ki = 0;
    public static final double climbPID_Kd = 0;

    public static final double climbPIDMinRange = 0;
    public static final double climbPIDMaxRange = 0;

    public static final double climbPIDMinOutput = 0;
    public static final double climbPIDMaxOutput = 0;

    public static final double climbOutAngle = 0;
    public static final double climbInAngle = 0;
  }

  public class LEDConstants {
    public static final int candle_ID = 46;

    public static final int ledNum = 32;

    public static boolean LEDFlag = false;
    public static boolean hasGamePiece = false;
    public static boolean intakeGamePiece = false;
    public static boolean tracking = false;
    public static boolean arrivePosition_Intake = false;
    public static boolean hasFrontRightTarget = false;
    public static boolean hasFrontLeftTarget = false;
    public static boolean canTrackLeft = false;
    public static boolean canTrackRight = false;
    public static boolean noTarget = false;
    public static boolean intakeArriving = false;
    public static boolean arrivePosition_Base = false;
    public static boolean shootGamePiece = false;
    public static boolean onCage = false;
    public static boolean climbing = false;
    public static boolean fireAnimation = false;
    public static boolean normal = false;


  }

  public static class Module_NeoConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;

    public static final double kModuleDistance = 22.24*0.0254;

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final double drivePidController_Kp = 0.2;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.13;
    public static final double driveFeedforward_Kv = 2.58;

  }

  public class Swerve_NeoConstants {
    public static final int rightFrontDrive_ID = 8;
    public static final int rightBackDrive_ID = 13;
    public static final int leftFrontDrive_ID = 1;
    public static final int leftBackDrive_ID = 19;

    public static final int rightFrontTurning_ID = 16;
    public static final int rightBackTurning_ID = 26;
    public static final int leftFrontTurning_ID = 7;
    public static final int leftBackTurning_ID = 29;

    public static final int rightFrontAbsolutedEncoder_ID = 41;
    public static final int rightBackAbsolutedEncoder_ID = 42;
    public static final int leftFrontAbsolutedEncoder_ID = 43;
    public static final int leftBackAbsolutedEncoder_ID = 44;

    public static final double leftFrontOffset = -0.106689;
    public static final double leftBackOffset = 0.380371;
    public static final double rightFrontOffset = -0.058837;
    public static final double rightBackOffset = 0.416503;

    public static final int gyro_ID = 55;

    public static final double kModuleDistance = 22.24*0.0254;
    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2));

    
    public static final double pathingMoving_Kp = 0;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;

    public static final double pathingtheta_Kp = 0;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 4.6;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 720;

  }
}
