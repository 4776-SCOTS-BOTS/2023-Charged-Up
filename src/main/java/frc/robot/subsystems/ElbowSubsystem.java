// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.customClass.ArmJointConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ElbowSubsystem extends TrapezoidProfileSubsystem {
  private SparkMaxPIDController m_motor;
  private final ArmFeedforward m_feedforward;
  private AbsoluteEncoder jointEncoder;
  private AbsoluteEncoder offsetEncoder;
  private double offsetAngleRads;

  /** Create a new ArmSubsystem. */
  public ElbowSubsystem(SparkMaxPIDController sparkMAX, ArmJointConstants jointConstants, 
  double initPosition, AbsoluteEncoder jointEncoder, AbsoluteEncoder offestEncoder) {
    super(
        jointConstants.trapConstraints,
        initPosition);

        m_feedforward =
        new ArmFeedforward(
            jointConstants.kSVolts, jointConstants.kGVolts,
            jointConstants.kVVoltSecondPerRad, jointConstants.kAVoltSecondSquaredPerRad);

    this.offsetEncoder = offestEncoder;
    this.jointEncoder = jointEncoder;

    this.offsetAngleRads = offsetEncoder.getPosition();

    this.m_motor = sparkMAX;

    m_motor.setP(jointConstants.kP);
    m_motor.setI(jointConstants.kI);
    m_motor.setD(jointConstants.kD);
    m_motor.setFF(jointConstants.kFF);
    m_motor.setOutputRange(jointConstants.kMinOutput, jointConstants.kMaxOutput);

  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(getFFAngle(setpoint.position), setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setReference(
        setpoint.position, CANSparkMax.ControlType.kPosition, 0, feedforward);

    // Comment out after tuning
    //SmartDashboard.putNumber("Elbow FF", feedforward);
    // SmartDashboard.putNumber("Elbow Setpoint", setpoint.position);
  }


  public Command setArmGoalCommand(double goal) {
    return Commands.runOnce(() -> setGoal(goal), this);
  }

  public Command holdArmPositionCommand() {
    return Commands.runOnce(() ->{ 
      holdArmPosition();
    }, this);
  }

  public void holdArmPosition(){
    setGoal(jointEncoder.getPosition());
    enable();
  }

  public double getOffset(){
      return offsetAngleRads;
  }

  public double getFFAngle(double setpoint){
    return Math.PI + getOffset() - setpoint;
  }

  public void setOffsetAngleRads(double offsetAngleRads) {
    this.offsetAngleRads = offsetAngleRads - Math.PI/2; //Need to compensate for shoulder zero point
  }

  

}
