// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends SubsystemBase {
  private final CANSparkMax shoulder1;
  private final CANSparkMax shoulder2;
  private final CANSparkMax elbow;

  private final AbsoluteEncoder elbowEncoder;
  private final AbsoluteEncoder shoulderEncoder;

  private final SparkMaxPIDController elbowPIDController;
  private final SparkMaxPIDController shoulderPIDController;

  private final ElbowSubsystem elbowTrapController;

  public double elbowPosition;
  public double shoulderPosition;

  /** Creates a new Arm. */
  public Arm() {
    shoulder1 = new CANSparkMax(Constants.ArmConstants.kShoulder1Port , MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.ArmConstants.kShoulder2Port , MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.kElbowPort , MotorType.kBrushless);

    shoulder1.setInverted(Constants.ArmConstants.kShoulder1Inv);
    shoulder2.setInverted(Constants.ArmConstants.kShoulder2Inv);

    shoulder2.follow(shoulder1, true);

    shoulder1.setIdleMode(IdleMode.kBrake);
    shoulder2.setIdleMode(IdleMode.kBrake);
    elbow.setIdleMode(IdleMode.kBrake);

    elbow.setSmartCurrentLimit(Constants.ArmConstants.Elbow.kCurrentLimit);

    elbowEncoder = elbow.getAbsoluteEncoder(Type.kDutyCycle);
    elbowPIDController = elbow.getPIDController();
    elbowPIDController.setFeedbackDevice(elbowEncoder);
    elbowPIDController.setPositionPIDWrappingEnabled(false);
    elbowEncoder.setPositionConversionFactor((2 * Math.PI));
    elbowEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);
    elbowEncoder.setZeroOffset(Constants.ArmConstants.elbow.kOffset);

    shoulderEncoder = shoulder1.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderPIDController = shoulder1.getPIDController();
    shoulderPIDController.setFeedbackDevice(shoulderEncoder);
    shoulderPIDController.setPositionPIDWrappingEnabled(false);
    shoulderEncoder.setPositionConversionFactor((2 * Math.PI));
    shoulderEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

    elbowTrapController = new ElbowSubsystem(elbowPIDController, Constants.ArmConstants.elbow,
        getElbowCurrentPos(), elbowEncoder, shoulderEncoder);
    elbowTrapController.disable();

    // Set the PID gains for the elbow motor.  These values are set in the Trapezoid Controller above.
    // elbowPIDController.setP(ArmConstants.Elbow.kP);
    // elbowPIDController.setI(ArmConstants.Elbow.kI);
    // elbowPIDController.setD(ArmConstants.Elbow.kD);
    // elbowPIDController.setFF(ArmConstants.Elbow.kFF);
    // elbowPIDController.setOutputRange(ArmConstants.Elbow.kMinOutput,
    // ArmConstants.Elbow.kMaxOutput);

    // Set the PID gains for the elbow motor.
    shoulderPIDController.setP(ArmConstants.Shoulder.kP);
    shoulderPIDController.setI(ArmConstants.Shoulder.kI);
    shoulderPIDController.setD(ArmConstants.Shoulder.kD);
    shoulderPIDController.setFF(ArmConstants.Shoulder.kFF);
    shoulderPIDController.setOutputRange(ArmConstants.Shoulder.kMinOutput,
        ArmConstants.Shoulder.kMaxOutput);

    Shuffleboard.getTab("Arm").addNumber("Elbow", this::getElbowPositionDeg);
    Shuffleboard.getTab("Arm").addNumber("Shoulder", this::getShoulderPositionDeg);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbowPosition = getElbowCurrentPos();
    shoulderPosition = getShoulderCurrentPos();
  }

  public void setElbowPosition(double position){
    //Should calibrate Absolute Encoder on SparkMAX to zero with elbow straight.
    if(position > ArmConstants.Elbow.kCWLimit*2*Math.PI && position < ArmConstants.Elbow.kCCWLimit*2*Math.PI){
      //Do nothing.  Invalid elbow position
    } else {
      elbowTrapController.enable();
      elbowTrapController.setGoal(position);
    }
  }

  public void setShoulderPosition(double position){
    //Should calibrate Absolute Encoder on SparkMAX to zero with arm straight up.
    if(position > ArmConstants.Shoulder.kCWLimit*2*Math.PI && position < ArmConstants.Shoulder.kCCWLimit*2*Math.PI){
      //Do nothing.  Invalid shoulder position
    } else {
      //shoulderPIDController.setReference(position, ControlType.kSmartMotion);
    }
  }

  public double getElbowCurrentPos(){
    return elbowEncoder.getPosition();
  }

  public double getElbowPosition(){
    return elbowPosition;
  }
  
  public double getElbowPositionDeg(){
    return Math.toDegrees(elbowPosition);
  }

  public double getShoulderCurrentPos(){
    return shoulderEncoder.getPosition();
  }

  public double getShoulderPosition(){
    return shoulderPosition;
  }

  public double getShoulderPositionDeg(){
    return Math.toDegrees(shoulderPosition);
  }

  public void runShoulder(double power){
    shoulder1.set(Constants.ArmConstants.Shoulder.kManualScale * power);
  }

  public void runElbow(double power){
    elbowTrapController.disable();
    elbow.set(Constants.ArmConstants.Elbow.kManualScale * power);
    elbowTrapController.setGoal(elbowPosition); //Keep trap controller updated with position
  }

  public void holdElbowPosition(){
    elbowTrapController.holdArmPosition();
  }

  
}
