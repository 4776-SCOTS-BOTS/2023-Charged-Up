// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.jsontype.impl.StdSubtypeResolver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.customClass.ArmJointConstants;
import frc.robot.customClass.CRGB;
import frc.robot.customClass.ArmPosition;

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

  public static final RobotType robotType = RobotType.PracticeBot;

  public static final class ConfigConstants {
    public static boolean fullShuffleBoardOutput = false;
    public static boolean hasCamera = true;

  }

  public static final CRGB kRGB_red = new CRGB(255, 0, 0);
  public static final CRGB kRGB_green = new CRGB(0, 255, 0);
  public static final CRGB kRGB_blue = new CRGB(0, 0, 255);
  public static final CRGB kRGB_yellow = new CRGB(204, 104, 0);
  public static final CRGB kRGB_purple = new CRGB(70, 0, 40);
  public static final CRGB kRGB_boaz = new CRGB(0, 59, 111);
  public static final CRGB kRGB_greenLow = new CRGB(0, 20, 0);

  public static final class DriveConstants {
    // Any constants that are not final can and should be update in
    // GenerateConstants
    // Non-final constants are initialized with the values of the practice bot
    // below.

    public static final double driveNormalPercentScale = 0.7;
    public static final double rotNormalRateModifier = 1.5;
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

    public static int kFrontLeftTurningAnalogPort = 0;
    public static int kRearLeftTurningAnalogPort = 3;
    public static int kFrontRightTurningAnalogPort = 1;
    public static int kRearRightTurningAnalogPort = 2;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static Rotation2d kFrontLeftTurningHome = new Rotation2d(Math.toRadians(0));
    public static Rotation2d kRearLeftTurningHome = new Rotation2d(Math.toRadians(0));
    public static Rotation2d kFrontRightTurningHome = new Rotation2d(Math.toRadians(0));
    public static Rotation2d kRearRightTurningHome = new Rotation2d(Math.toRadians(0));

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

    public static double kPModuleTurningController = 2.5;
    public static double kDModuleTurningController = 0;

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
    public static final double kMaxSpeedMetersPerSecond = 1.0;// was 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;// was 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// was Pi
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
    public static final int kShoulder2Port = 20;
    public static final boolean kShoulder2Inv = false;
    public static final int kElbowPort = 22;

    public static final ArmPosition PICKUP_POSITION = new ArmPosition(75, 145);
    public static final ArmPosition SAFE_POSITION = new ArmPosition(30, 205.0);
    public static final ArmPosition HIGH_POSITION = new ArmPosition(180, 235.0);
    public static final ArmPosition MID_POSITION = new ArmPosition(290.0, 195.0);
    public static final ArmPosition LOW_POSITION = new ArmPosition(290, 270.0);

    public static final ArmPosition CUBE_HIGH_POSITION = new ArmPosition(180, 235.0);

    // Elbow Constants

    public static final class Elbow {
      public static final double kSVolts = 0;
      public static final double kGVolts = 0.4;
      public static final double kVVoltSecondPerRad = 2.92;
      public static final double kAVoltSecondSquaredPerRad = 0;
      public static final double kP = 0.3;
      public static final double kI = 0;
      public static final double kD = 0.06;
      public static final double kFF = 0;
      public static final double kMinOutput = -0.6;
      public static final double kMaxOutput = 0.6;
      public static final double kUpperLimit = Math.toRadians(300);
      public static final double kLowerLimit = Math.toRadians(10);
      public static final double kMaxVelocityRadPerSecond = Math.toRadians(90);
      public static final double kMaxAccelerationRadPerSecSquared = Math.toRadians(180);
      public static final double kOffset = 4.60-Math.toRadians(55); // Radians
      public static final int kCurrentLimit = 10;
      public static final double kManualScale = 0.3;

      public static final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(
          kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared);

    }

    public static final ArmJointConstants elbow = new ArmJointConstants(Elbow.kSVolts, Elbow.kGVolts,
        Elbow.kVVoltSecondPerRad, Elbow.kAVoltSecondSquaredPerRad,
        Elbow.kP, Elbow.kI, Elbow.kD, Elbow.kFF, Elbow.kMinOutput, Elbow.kMaxOutput, Elbow.kUpperLimit, Elbow.kLowerLimit,
        Elbow.trapConstraints, Elbow.kOffset, 0);

    // Shoulder Constants
    public static final class Shoulder {
      public static final double kSVolts = 0;
      public static final double kGAplhaVolts = 0.35;
      public static final double kGBetaVolts = 0.50;
      public static final double kVVoltSecondPerRad = 1.95;
      public static final double kAVoltSecondSquaredPerRad = 0;
      public static final double kP = 0.4;
      public static final double kI = 0;
      public static final double kD = 0.06;
      public static final double kFF = 0;
      public static final double kMinOutput = -0.6;
      public static final double kMaxOutput = 0.6;
      public static final double kUpperLimit = Math.toRadians(340);
      public static final double kLowerLimit = Math.toRadians(20);
      public static final double kMaxVelocityRadPerSecond = Math.toRadians(90);
      public static final double kMaxAccelerationRadPerSecSquared = Math.toRadians(180);
      public static final double kOffset = 1.88 - Math.PI/2 + Math.toRadians(0); // Adjusted to zero degrees straight down
      public static final int kCurrentLimit = 30;
      public static final double kManualScale = 0.3;

      public static final TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(
          kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared);
    }

    public static final ArmJointConstants shoulder = new ArmJointConstants(Shoulder.kSVolts, Shoulder.kGAplhaVolts,
        Shoulder.kVVoltSecondPerRad, Shoulder.kAVoltSecondSquaredPerRad,
        Shoulder.kP, Shoulder.kI, Shoulder.kD, Shoulder.kFF, Shoulder.kMinOutput, Shoulder.kMaxOutput, Shoulder.kUpperLimit, Shoulder.kLowerLimit,
        Shoulder.trapConstraints, Shoulder.kOffset, Shoulder.kGBetaVolts);

  }

  // Solenoid Constants RAAAAAAA
  public static final class PneumaticsConstants {
    public static final int phCanID = 10;
    public static final int gripperSolenoidPort = 8;
    public static final int intakeSolenoidPort = 0;
    public static final int kickerSolenoidPort = 11;
  }

  // Intake Constants
  public static final class IntakeConstants {
    public static final int kIntakePort = 23;
    public static final boolean kIntakeInv = false;
    public static final double kIntakePower = 0.85; // @.60 power movement forward is needed to collect cube

    public static final double kIntakePowerCone = 0.90;
    public static final double kIntakePowerCube = 0.50;

    // Magic Carpet Constants
    public static final int kMagicCarpetPort = 24;
    public static final boolean kMagicCarpetInv = true;
    public static final double kMagicCarpetPower = 0.25;

  }

  public static final class LEDConstants {
    // LED Port
    public static final int kLEDPort = 0;
    public static final int kStrandLength = 33;
  }

  public static RobotType GenerateConstants(RobotType robot) {
    switch (robot) {
      case CompBot: {
        ConfigConstants.hasCamera = false;

        // Swerve Module Alignment
        DriveConstants.kFrontLeftTurningHome = new Rotation2d(Math.toRadians(-193+180));
        DriveConstants.kRearLeftTurningHome = new Rotation2d(Math.toRadians(-48+180));
        DriveConstants.kFrontRightTurningHome = new Rotation2d(Math.toRadians(63));
        DriveConstants.kRearRightTurningHome = new Rotation2d(Math.toRadians(148));

        //Encoder Ports
        DriveConstants.kFrontLeftTurningAnalogPort = 55;
        DriveConstants.kRearLeftTurningAnalogPort = 59;
        DriveConstants.kFrontRightTurningAnalogPort = 53;
        DriveConstants.kRearRightTurningAnalogPort = 57;

        ModuleConstants.kPModuleTurningController = 1.0;
        ModuleConstants.kDModuleTurningController = 0;

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
        ConfigConstants.hasCamera = true;

        DriveConstants.kFrontLeftTurningHome = new Rotation2d(Math.toRadians(-10.5));
        DriveConstants.kRearLeftTurningHome = new Rotation2d(Math.toRadians(-136.8));
        DriveConstants.kFrontRightTurningHome = new Rotation2d(Math.toRadians(-51.6));
        DriveConstants.kRearRightTurningHome = new Rotation2d(Math.toRadians(-156.2));

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
