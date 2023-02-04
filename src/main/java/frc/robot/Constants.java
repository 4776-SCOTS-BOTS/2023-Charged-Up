// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.customClass.ArmJointConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum RobotType {
    CompBot,
    PracticeBot
  }


  public static final RobotType robotType = RobotType.CompBot;

  public static final class ConfigConstants{
    public static boolean fullShuffleBoardOutput = false;
    public static boolean hasCamera = true;
  }

  public static final class DriveConstants {
    // Any constants that are not final can and should be update in GenerateConstants
    // Non-final constants are initialized with the values of the practice bot below.


    public static final double driveNormalPercentScale = 0.7;
    public static final double rotNormalRateModifier = 1.25;  
    public static final double driveLowPercentScale = 0.5;
    public static final double rotLowRateModifier = 0.75;    


    public static double drivePercentScale = driveNormalPercentScale;
    public static double rotRateModifier = rotNormalRateModifier;

    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kRearLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 5;
    public static final int kRearLeftTurningMotorPort = 9;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] { 0, 1 };
    public static final int[] kRearLeftTurningEncoderPorts = new int[] { 2, 3 };
    public static final int[] kFrontRightTurningEncoderPorts = new int[] { 6, 7 };
    public static final int[] kRearRightTurningEncoderPorts = new int[] { 8, 9 };

    public static final int kFrontLeftTurningAnalogPort = 2;
    public static final int kRearLeftTurningAnalogPort = 1;
    public static final int kFrontRightTurningAnalogPort = 3;
    public static final int kRearRightTurningAnalogPort = 0;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static Rotation2d kFrontLeftTurningHome = new Rotation2d(Math.toRadians(7.7));
    public static Rotation2d kRearLeftTurningHome = new Rotation2d(Math.toRadians(-19.2));
    public static Rotation2d kFrontRightTurningHome = new Rotation2d(Math.toRadians(+37.5));
    public static Rotation2d kRearRightTurningHome = new Rotation2d(Math.toRadians(-134.5));

    // Distance between centers of right and left wheels on robot
    public static double kTrackWidth = 0.587375;
    // Distance between front and back wheels on robot
    public static double kWheelBase = 0.47; // actually is 0.4953 and was using 0.587375 021222 ;

    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = true;
    public static final double kMaxSpeedMetersPerSecond = 3.35; // Was 1 021222

  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final int kTurningEncoderCPR = 415;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kTurningEncoderCPR;

    public static final double kMaxRPM = 5700;
    public static final double kWheelDiameter = 0.102;
    public static final double kMotorGearsToWheelGears = 6.67;
    public static final double kRevolutionsToMeters = Math.PI * kWheelDiameter / kMotorGearsToWheelGears;
    public static final double kRPMToMetersPerSecond = Math.PI * kWheelDiameter / (60 * kMotorGearsToWheelGears);


    public static final double kPModuleTurningController = 2.5;
    public static final double kDModuleTurningController = 0;

    public static final double kPModuleDriveController = 0.6;
    private static final double kDriveP = 15.0;
    private static final double kDriveI = 0.01;
    private static final double kDriveD = 0.1;
    private static final double kDriveF = 0.2;

    public static final double kFrontLeftTurningEncoderCounts = 2 * Math.PI / 415.1;
    public static final double kFrontRightTurningEncoderCounts = 2 * Math.PI / 415.6;
    public static final double kRearLeftTurningEncoderCounts = 2 * Math.PI / 415.2;
    public static final double kRearRightTurningEncoderCounts = 2 * Math.PI / 415.7;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.0;//was 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;//was 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;//was Pi
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ArmConstants {
    public static final int kShoulder1Port = 21;
    public static final boolean kShoulder1Inv = true;
    public static final int kShoulder2Port = 20; // TODO: Make these the actual port numbers
    public static final boolean kShoulder2Inv = false;
    public static final int kElbowPort = 22;

    // Elbow Constants

    public static final class Elbow {
      public static final double kSVolts = 0;
      public static final double kGVolts = 0;
      public static final double kVVoltSecondPerRad = 0;
      public static final double kAVoltSecondSquaredPerRad = 0;
      public static final double kP = 0.001;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0;
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
      public static final double kCCWLimit = 0.7;
      public static final double kCWLimit = 0.3;
      public static final double kMaxVelocityRadPerSecond = Math.toRadians(90);
      public static final double kMaxAccelerationRadPerSecSquared = Math.toRadians(180);
      public static final double kOffset = 0.779; // Final constant is 0-1 from Rev Throughbore setup
      public static final int kCurrentLimit = 10;
      public static final double kManualScale = 0.5;

      public static final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared);

    }

    public static final ArmJointConstants elbow = new ArmJointConstants(Elbow.kSVolts, Elbow.kGVolts,
        Elbow.kVVoltSecondPerRad, Elbow.kAVoltSecondSquaredPerRad,
        Elbow.kP, Elbow.kI, Elbow.kD, Elbow.kFF, Elbow.kMinOutput, Elbow.kMaxOutput, Elbow.kCCWLimit, Elbow.kCWLimit,
        Elbow.trapConstraints, Elbow.kOffset);

    // Shoulder Constants
    public static final class Shoulder {
      public static final double kP = 0.04;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0.01;
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
      public static final double kCCWLimit = 0.65;
      public static final double kCWLimit = 0.35;

      public static final double kManualScale = 0.5;
    }

  }
//Solenoid Constants RAAAAAAA
  public static final class PneumaticsConstants{
    public static final int phCanID = 10;
    public static final int gripperSolenoidPort = 1;
    public static final int intakeSolenoidPort = 10;
  }

  public static RobotType GenerateConstants(RobotType robot) {
    switch (robot) {
        case CompBot: {
          ConfigConstants.hasCamera = false;

          //Swerve Module Alignment
          DriveConstants.kFrontLeftTurningHome = new Rotation2d(Math.toRadians(+165.1));
          DriveConstants.kRearLeftTurningHome = new Rotation2d(Math.toRadians(-96.7));
          DriveConstants.kFrontRightTurningHome = new Rotation2d(Math.toRadians(+77.7));
          DriveConstants.kRearRightTurningHome = new Rotation2d(Math.toRadians(-156.8));//80.8

          // Distance between centers of right and left wheels on robot
          double kTrackWidth = DriveConstants.kTrackWidth = 0.584;
          // Distance between front and back wheels on robot
          double kWheelBase = DriveConstants.kWheelBase = 0.56; // actually is 0.4953 and was using 0.587375 021222 ;

          DriveConstants.kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        }
          break;

        case PracticeBot: {
          ConfigConstants.hasCamera = false;
          
          DriveConstants.kFrontLeftTurningHome = new Rotation2d(Math.toRadians(7.7));
          DriveConstants.kRearLeftTurningHome = new Rotation2d(Math.toRadians(-19.2));
          DriveConstants.kFrontRightTurningHome = new Rotation2d(Math.toRadians(+37.5));
          DriveConstants.kRearRightTurningHome = new Rotation2d(Math.toRadians(-58.6));//-134.5

          // Distance between centers of right and left wheels on robot
          double kTrackWidth = DriveConstants.kTrackWidth = 0.587375;
          // Distance between front and back wheels on robot
          double kWheelBase = DriveConstants.kWheelBase = 0.47; // actually is 0.4953 and was using 0.587375 021222 ;

          DriveConstants.kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        }
          break;
      }
      return robot;
    }



  }

  