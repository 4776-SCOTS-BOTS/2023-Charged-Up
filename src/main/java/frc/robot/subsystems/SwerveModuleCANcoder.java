// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

public class SwerveModuleCANcoder {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final boolean InvertLeft;
  private final boolean InvertBack;
  private final RelativeEncoder m_driveEncoder;
  private final WPI_CANCoder m_turningEncoder;
  private final SparkMaxPIDController m_drivePID;
  public CANCoderConfiguration swerveCANCoderConfig;
  private double turningEncoderCounts;
  private int turningMotorChannel;

  private final PIDController m_drivePIDController = new PIDController(0.2, 0.1, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      ModuleConstants.kDModuleTurningController,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  PIDController turningPID = new PIDController(ModuleConstants.kPModuleTurningController, 0,
      ModuleConstants.kDModuleTurningController);


  /**
   * Constructs a SwerveModule.
   * 
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleCANcoder(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPort,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      boolean is_invertedLeft,
      boolean is_invertedBack,
      Rotation2d homeLoc) {

    this.turningMotorChannel = turningMotorChannel;
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setInverted(is_invertedLeft);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setInverted(true);

    this.m_driveEncoder = m_driveMotor.getEncoder();
    // m_driveEncoder.setInverted(is_invertedLeft);
    this.m_turningEncoder = new WPI_CANCoder(turningEncoderPort, "rio");

    m_drivePID = m_driveMotor.getPIDController();
    m_drivePID.setP(ModuleConstants.kDriveP);
    m_drivePID.setI(ModuleConstants.kDriveI);
    m_drivePID.setD(ModuleConstants.kDriveD);
    m_drivePID.setFF(ModuleConstants.kDriveFF);
    m_drivePID.setOutputRange(-1, 1);

    /* Swerve CANCoder Configuration */
    this.swerveCANCoderConfig = new CANCoderConfiguration();
    swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    swerveCANCoderConfig.sensorDirection = !turningEncoderReversed;
    swerveCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    swerveCANCoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
    swerveCANCoderConfig.unitString = "rad";
    swerveCANCoderConfig.magnetOffsetDegrees = homeLoc.getDegrees();
    m_turningEncoder.configAllSettings(swerveCANCoderConfig);

    InvertLeft = is_invertedLeft;
    InvertBack = is_invertedBack;
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kRevolutionsToMeters);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kRPMToMetersPerSecond);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);


    // Limit the PID Controller's input range between -4 and 4 and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);

    Shuffleboard.getTab("SwerveTurning").addNumber("SwerveModuleTurning" + turningMotorChannel, this::getAngleRadians);
    // Shuffleboard.getTab("Swerve").addNumber("SwerveModule"+driveMotorChannel,
    // m_driveEncoder::getPosition);
    Shuffleboard.getTab("SwerveDrive").addNumber("SwerveModule" + driveMotorChannel, this::getDrivePosition);
    resetEncoders();
  }

  public double getAngleRadians() {
    return m_turningEncoder.getAbsolutePosition();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAngleRadians()));
    // return new SwerveModuleState(m_driveEncoder.getVelocity(), new
    // Rotation2d(m_turningEncoder.get()));
  }

    /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(), new Rotation2d(getAngleRadians()));
  }

  public double getDrivePosition() {
    return (InvertLeft ? -1 : 1) * m_driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return (InvertLeft ? -1 : 1) * m_driveEncoder.getVelocity();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean noMovement, boolean noOpt) {
    if (noMovement) {
      // no Movement
      m_driveMotor.set(0);
      m_turningMotor.set(0);
    } else {
      // There is movement

      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state;
      if (!noOpt) {
        state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngleRadians()));
      } else {
        state = desiredState;
      }
      
      // Calculate the drive output based on purely percent scaling of max speed.
       final double driveOutput = (InvertLeft ? -1 : 1) * state.speedMetersPerSecond
           / DriveConstants.kMaxSpeedMetersPerSecond;


      //Apply inversion to state if needed
      state.speedMetersPerSecond = (InvertLeft ? -1 : 1) * state.speedMetersPerSecond;


      // Calculate the turning motor output from the turning PID controller.
      // Turning power is always negated because + motor power is clockwise regardless of module
      // orientation and + turning power is requesting couter-clockwise rotation
      final var turnOutput =
          // m_turningPIDController.calculate(getAngleRadians(),
          // state.angle.getRadians());
          - turningPID.calculate(getAngleRadians(), state.angle.getRadians());
      
      //     var error = Math.abs((state.angle.getRadians() % (2 * Math.PI)) - (getAngleRadians() % (2 * Math.PI)));
      // error = Math.min(error, 2 * Math.PI - error);

      // var ready = 1 - error * 2 / Math.PI;
      // ready = Math.max(ready, 0);

      //turningPID.calculate(measurement)

      // Set drive output
      if (ModuleConstants.useSparkMaxPID) {
        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
      } else {
        m_driveMotor.set(driveOutput);
      }
      m_turningMotor.set(turnOutput);
    }
  }

  /** Zeros all the SwerveModule encoders. */
  // resetEncoders does not remove field oriented drive
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    //m_turningEncoder.reset();  Do not want to reset the Absolute encoder
  }

  public double getDriveMotorCurrent(){
    return m_driveMotor.getOutputCurrent();
  }

}
