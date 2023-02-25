// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import java.nio.channels.Selector;
import java.sql.Driver;

import frc.robot.subsystems.SwerveModuleCANcoder;

import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
import frc.robot.Constants.ConfigConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules

  //  private final SwerveModuleCANcoder m_frontLeft = new SwerveModuleCANcoder(
  //     DriveConstants.kFrontLeftDriveMotorPort,
  //     DriveConstants.kFrontLeftTurningMotorPort,
  //     DriveConstants.kFrontLeftTurningAnalogPort,
  //     DriveConstants.kFrontLeftDriveEncoderReversed,
  //     DriveConstants.kFrontLeftTurningEncoderReversed,
  //     false, false,
  //     DriveConstants.kFrontLeftTurningHome);

  //  private final SwerveModuleCANcoder m_rearLeft = new SwerveModuleCANcoder(
  //      DriveConstants.kRearLeftDriveMotorPort,
  //      DriveConstants.kRearLeftTurningMotorPort,
  //      DriveConstants.kRearLeftTurningAnalogPort,
  //      DriveConstants.kRearLeftDriveEncoderReversed,
  //      DriveConstants.kRearLeftTurningEncoderReversed,
  //      false, false,
  //      DriveConstants.kRearLeftTurningHome);

  //  private final SwerveModuleCANcoder m_frontRight = new SwerveModuleCANcoder(
  //      DriveConstants.kFrontRightDriveMotorPort,
  //      DriveConstants.kFrontRightTurningMotorPort,
  //      DriveConstants.kFrontRightTurningAnalogPort,
  //      DriveConstants.kFrontRightDriveEncoderReversed,
  //      DriveConstants.kFrontRightTurningEncoderReversed,
  //      false, false,
  //      DriveConstants.kFrontRightTurningHome);

  //  private final SwerveModuleCANcoder m_rearRight = new SwerveModuleCANcoder(
  //      DriveConstants.kRearRightDriveMotorPort,
  //      DriveConstants.kRearRightTurningMotorPort,
  //      DriveConstants.kRearRightTurningAnalogPort,
  //      DriveConstants.kRearRightDriveEncoderReversed,
  //      DriveConstants.kRearRightTurningEncoderReversed,
  //      false, false,
  //      DriveConstants.kRearRightTurningHome);

  //  private final SwerveModuleCANcoder[] swerveModules = {
  //      m_frontLeft,
  //      m_frontRight,
  //      m_rearLeft,
  //      m_rearRight };

  private final SwerveModuleAbs m_frontLeft = new SwerveModuleAbs(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningAnalogPort,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      false, false,
      DriveConstants.kFrontLeftTurningHome);

  private final SwerveModuleAbs m_rearLeft = new SwerveModuleAbs(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningAnalogPort,
      DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftTurningEncoderReversed,
      false, false,
      DriveConstants.kRearLeftTurningHome);

  private final SwerveModuleAbs m_frontRight = new SwerveModuleAbs(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningAnalogPort,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      false, false,
      DriveConstants.kFrontRightTurningHome);

  private final SwerveModuleAbs m_rearRight = new SwerveModuleAbs(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningAnalogPort,
      DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightTurningEncoderReversed,
      false, false,
      DriveConstants.kRearRightTurningHome);

  private final SwerveModuleAbs[] swerveModules = {
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight };

  // The gyro sensor
  // private final Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);
  //private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Zeroed Module State
  private SwerveModuleState zeroState = new SwerveModuleState();
  
  // Init Limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
  new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_rearLeft.getPosition(),
    m_rearRight.getPosition()
  });

  private GenericEntry[] swerveModuleShuffleTargetAngle = new GenericEntry[4];
  private GenericEntry[] swerveModuleShuffleTargetSpeed = new GenericEntry[4];
  private GenericEntry[] swerveModuleShuffleActualAngle = new GenericEntry[4];
  private GenericEntry[] swerveModuleShuffleActualSpeed = new GenericEntry[4];
  private GenericEntry[] swerveModuleShuffleTurnVolts = new GenericEntry[4];

  private GenericEntry gyroAngle;

  //private NetworkTableEntry odoX, odoY, odoRot; /2022 version
  private GenericEntry odoX, odoY, odoRot;
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.enableLogging(true);
    //m_gyro.enableBoardlevelYawReset(true);
    setupShuffleBoard();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        new Rotation2d(Math.toRadians(getHeading())),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    odoX.setDouble(this.getPose().getX());
    odoY.setDouble(this.getPose().getY());
    odoRot.setDouble(this.getPose().getRotation().getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    },
    pose);
  }

  public void zeroOdometry(){
    
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    // System.out.println("State: "+swerveModuleStates[0].angle.getRadians()+",
    // drive: "+xSpeed+" or "+ySpeed+" when at "+m_frontLeft.getAngleRadians());

    boolean noMovement = xSpeed == 0 && ySpeed == 0 && rot == 0;  //Renable on 3/9

    
      for (int i = 0; i < 4; i++) {
        swerveModuleShuffleTargetAngle[i].setDouble(swerveModuleStates[i].angle.getDegrees());
        swerveModuleShuffleTargetSpeed[i].setDouble(swerveModuleStates[i].speedMetersPerSecond);

        swerveModules[i].setDesiredState(swerveModuleStates[i], noMovement, false);

        swerveModuleShuffleActualAngle[i].setDouble(swerveModules[i].getState().angle.getDegrees());
        swerveModuleShuffleActualSpeed[i].setDouble(swerveModules[i].getState().speedMetersPerSecond);

        //swerveModuleShuffleTurnVolts[i].setDouble(swerveModules[i].getRawVolts());

      }

  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0], false, false);
    m_frontRight.setDesiredState(desiredStates[1], false, false);
    m_rearLeft.setDesiredState(desiredStates[2], false, false);
    m_rearRight.setDesiredState(desiredStates[3], false, false);
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    System.out.println("Zeroing");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -m_gyro.getAngle();
    //return m_gyro.getRotation2d().getDegrees();
  }

  public Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void homeAllModules(){
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setDesiredState(zeroState, false, true);
    }
  
  }
  
  private void setupShuffleBoard() {
    final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
    //ShuffleboardTab smartDash = Shuffleboard.getTab("SmartDashboard");


    //Shuffleboard.getTab("SmartDashboard").add(m_gyro);

    // ShuffleboardLayout swerveAngleLayout = swerveTab
    // .getLayout("Swerve Angles", BuiltInLayouts.kList)
    // .withSize(6, 10)
    // .withPosition(6, 0);

    // ShuffleboardLayout swerveSpeedLayout = swerveTab
    // .getLayout("Swerve Speed", BuiltInLayouts.kList)
    // .withSize(4, 8)
    // .withPosition(14, 0);

    odoX =  swerveTab.add("Odometry X Position", this.getPose().getX()).getEntry();
    odoY = swerveTab.add("Odometry Y Position", this.getPose().getY()).getEntry();
    odoRot = swerveTab.add("Odometry Rotation", this.getPose().getRotation().getDegrees()).getEntry();

    
    gyroAngle = swerveTab.add("Gyro Heading", 0)
      .withSize(5,5)
      .withPosition(10, 10)
      .withWidget("Gyro This")
      .getEntry();


    for (int i = 0; i < 4; i++) {
      swerveModuleShuffleTargetAngle[i] = swerveTab.add("M" + i + " TarAngle", 0)
          .withSize(3, 2)
          .withPosition(6, i * 2)
          .getEntry();
      swerveModuleShuffleTargetSpeed[i] = swerveTab.add("M" + i + " TarSpeed", 0)
          .withSize(3, 2)
          .withPosition(12, i * 2)
          .getEntry();
      swerveModuleShuffleActualAngle[i] = swerveTab.add("M" + i + " ActAngle", 0)
          .withSize(3, 2)
          .withPosition(9, i * 2)
          .getEntry();
      swerveModuleShuffleActualSpeed[i] = swerveTab.add("M" + i + " ActSpeed", 0)
          .withSize(3, 2)
          .withPosition(15, i * 2)
          .getEntry();
      swerveModuleShuffleTurnVolts[i] = swerveTab.add("M" + i + " TurnV", 0)
          .withSize(3, 2)
          .withPosition(18, i * 2)
          .getEntry();
    }
  }

  public void setNormalDrive(){
    DriveConstants.drivePercentScale = DriveConstants.driveNormalPercentScale;
    DriveConstants.rotRateModifier = DriveConstants.driveNormalPercentScale;
  }

  public void setSlowDrive(){
    DriveConstants.drivePercentScale = DriveConstants.driveLowPercentScale;
    DriveConstants.rotRateModifier = DriveConstants.driveLowPercentScale;
  }
  
  /** //TODO: Work on finishing Limelight code w/ help
  public void autoAim(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tx>").getDouble(0);

    float heading_error = -tx;
        float steering_adjust = 0.0f;
        if (tx > 1.0)
        {
                steering_adjust = Kp*heading_error - min_command;
        }
        else if (tx < 1.0)
        {
                steering_adjust = Kp*heading_error + min_command;
        }
        left_command += steering_adjust;
        right_command -= steering_adjust;
  }
  */

  public void turnByAngle(double turnByDegrees){
    double goal = Math.toRadians(-m_gyro.getAngle() + turnByDegrees);
    Timer timer = new Timer();
    double timeout = 3.0;

    var thetaController = new ProfiledPIDController(2.5, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    thetaController.setTolerance(Math.toRadians(1));
    thetaController.setGoal(goal);

    timer.reset();
    timer.start();
    double rotation = 0;

    while (!thetaController.atGoal() && !timer.hasElapsed(timeout)) {
      System.out.println("Target: " + goal + " / Current: " + Math.toRadians(-m_gyro.getAngle()) + " / Time: " + timer.get());
      rotation = thetaController.calculate(Math.toRadians(-m_gyro.getAngle()));
      drive(0, 0, rotation, false);

    }

    drive(0, 0, 0, false);

  }

  public void coastModuleTurn(){
    m_frontLeft.setDesiredState(new SwerveModuleState(), true, false);
    m_frontRight.setDesiredState(new SwerveModuleState(), true, false);
    m_rearLeft.setDesiredState(new SwerveModuleState(), true, false);
    m_rearRight.setDesiredState(new SwerveModuleState(), true, false);
  }

  public void setXModuleState(){
    m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))), false, false);
    m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45))), false, false);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45))), false, false);
    m_rearRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))), false, false);
  }

}
