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
  public final CANSparkMax shoulder1;
  public final CANSparkMax shoulder2;
  public final CANSparkMax elbow;

  private final AbsoluteEncoder elbowEncoder;
  private final AbsoluteEncoder shoulderEncoder;

  private final SparkMaxPIDController elbowPIDController;
  private final SparkMaxPIDController shoulderPIDController;

  /** Creates a new Arm. */
  public Arm() {
    shoulder1 = new CANSparkMax(Constants.ArmConstants.kShoulder1Port , MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.ArmConstants.kShoulder2Port , MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.kElbowPort , MotorType.kBrushless);

    shoulder2.follow(shoulder1, true);

    shoulder1.setIdleMode(IdleMode.kBrake);
    shoulder2.setIdleMode(IdleMode.kBrake);
    elbow.setIdleMode(IdleMode.kBrake);

    elbowEncoder = elbow.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderEncoder = shoulder1.getAbsoluteEncoder(Type.kDutyCycle);
    elbowPIDController = elbow.getPIDController();
    shoulderPIDController = shoulder1.getPIDController();
    elbowPIDController.setFeedbackDevice(elbowEncoder);
    shoulderPIDController.setFeedbackDevice(shoulderEncoder);


    // Set the PID gains for the elbow motor.
    elbowPIDController.setP(ArmConstants.Elbow.kP);
    elbowPIDController.setI(ArmConstants.Elbow.kI);
    elbowPIDController.setD(ArmConstants.Elbow.kD);
    elbowPIDController.setFF(ArmConstants.Elbow.kFF);
    elbowPIDController.setOutputRange(ArmConstants.Elbow.kMinOutput,
        ArmConstants.Elbow.kMaxOutput);

    // Set the PID gains for the elbow motor.
    shoulderPIDController.setP(ArmConstants.Shoulder.kP);
    shoulderPIDController.setI(ArmConstants.Shoulder.kI);
    shoulderPIDController.setD(ArmConstants.Shoulder.kD);
    shoulderPIDController.setFF(ArmConstants.Shoulder.kFF);
    shoulderPIDController.setOutputRange(ArmConstants.Shoulder.kMinOutput,
        ArmConstants.Shoulder.kMaxOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElbowPosition(double position){
    //Should calibrate Absolute Encoder on SparkMAX to zero with elbow straight.
    if(position > ArmConstants.Elbow.kCWLimit && position < ArmConstants.Elbow.kCCWLimit){
      //Do nothing.  Invalid elbow position
    } else {
      elbowPIDController.setReference(position, ControlType.kSmartMotion);
    }
  }

  public void setShoulderPosition(double position){
    //Should calibrate Absolute Encoder on SparkMAX to zero with arm straight up.
    if(position > ArmConstants.Shoulder.kCWLimit && position < ArmConstants.Shoulder.kCCWLimit){
      //Do nothing.  Invalid shoulder position
    } else {
      shoulderPIDController.setReference(position, ControlType.kSmartMotion);
    }
  }

  public double getElbowCurrentPos(){
    return elbowEncoder.getPosition();
  }

  public double getShoulderCurrentPos(){
    return shoulderEncoder.getPosition();
  }
}
