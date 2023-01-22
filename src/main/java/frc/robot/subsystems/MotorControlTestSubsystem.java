// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

public class MotorControlTestSubsystem extends SubsystemBase {
  private final CANSparkMax motor;

  /** Creates a new MotorControlTestSubsystem. */
  public MotorControlTestSubsystem(int motorChannel, boolean useForwardLimit, boolean useReverseLimit) {
    this.motor = new CANSparkMax(motorChannel, MotorType.kBrushless);
    this.motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(useForwardLimit);
    this.motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(useReverseLimit);


  }

  public boolean checkForwardLimit(){

    return motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
