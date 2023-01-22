// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends SubsystemBase {
  public final CANSparkMax shoulder1 = new CANSparkMax(Constants.DriveConstants.kShoulder1Port , MotorType.kBrushless);
  public final CANSparkMax shoulder2 = new CANSparkMax(Constants.DriveConstants.kShoulder2Port , MotorType.kBrushless);
  public final CANSparkMax elbow = new CANSparkMax(Constants.DriveConstants.kElbowPort , MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
