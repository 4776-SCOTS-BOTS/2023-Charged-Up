// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
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

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax magicCarpetMotor;
  private boolean isRunning = false;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakePort, MotorType.kBrushless);
    intakeMotor.setInverted(true);

    magicCarpetMotor = new CANSparkMax(Constants.IntakeConstants.kIntakePort, MotorType.kBrushless);
    magicCarpetMotor.setInverted(true);
  }

  public void runIntake(double power){
    intakeMotor.set(power);
    isRunning = true;
  }
  public void intakeIn(){
    intakeMotor.set(IntakeConstants.intakePower);
    isRunning = true;
  }
  public void intakeOut(){
    intakeMotor.set(-IntakeConstants.intakePower);
    isRunning = true;
  }
  public void intakeOff(){
    intakeMotor.stopMotor();
    isRunning = false;
  }

  public void runMagicCarpet(double power){
    magicCarpetMotor.set(power);
    isRunning = true;
  }
  public void magicCarpetIn(){
    magicCarpetMotor.set(IntakeConstants.intakePower);
    isRunning = true;
  }
  public void magicCarpetOff(){
    magicCarpetMotor.stopMotor();
    isRunning = false;
  }
  
  public boolean getIsRunning(){
    return isRunning;
  }
}

