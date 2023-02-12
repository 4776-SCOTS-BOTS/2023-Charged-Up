// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.customClass;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ArmJointConstants {
  public double kSVolts, kGVolts, kVVoltSecondPerRad, kAVoltSecondSquaredPerRad, kgBeta;
  public double kP, kI, kD, kFF, kMinOutput, kMaxOutput, kCCWLimit, kCWLimit;
  public TrapezoidProfile.Constraints trapConstraints;
  public double kOffset;

  public ArmJointConstants(double kSVolts, double kGVolts, double kVVoltSecondPerRad, double kAVoltSecondSquaredPerRad,
      double kP, double kI, double kD, double kFF, double kMinOutput, double kMaxOutput, double kCCWLimit,
      double kCWLimit, TrapezoidProfile.Constraints trapConstraints, double kOffset, double kgBeta) {
    this.kSVolts = kSVolts;
    this.kGVolts = kGVolts;
    this.kVVoltSecondPerRad = kVVoltSecondPerRad;
    this.kAVoltSecondSquaredPerRad = kAVoltSecondSquaredPerRad;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kFF = kFF;
    this.kMinOutput = kMinOutput;
    this.kMaxOutput = kMaxOutput;
    this.kCCWLimit = kCCWLimit;
    this.kCWLimit = kCWLimit;
    this.trapConstraints = trapConstraints;
    this.kOffset = kOffset;
    this.kgBeta = kgBeta;
  }
}
