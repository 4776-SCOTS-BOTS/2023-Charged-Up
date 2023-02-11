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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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
import frc.robot.Constants.PneumaticsConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax magicCarpetMotor;
  private Solenoid intakeSolenoid;
  private boolean isRunning = false;
  private double intakePowerSetPoint = Constants.IntakeConstants.kIntakePowerCone;



  private enum IntakeState {
    IN, OUT, STOPPED;
  }
  private IntakeState intakeState = IntakeState.STOPPED;

  /** Creates a new Intake. */
  public Intake() {
    
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakePort, MotorType.kBrushless);
    intakeMotor.setInverted(Constants.IntakeConstants.kIntakeInv);
    
    magicCarpetMotor = new CANSparkMax(Constants.IntakeConstants.kMagicCarpetPort, MotorType.kBrushless);
    magicCarpetMotor.setInverted(Constants.IntakeConstants.kMagicCarpetInv);

    intakeSolenoid = new Solenoid(PneumaticsConstants.phCanID, PneumaticsModuleType.REVPH,PneumaticsConstants.intakeSolenoidPort);
  }

  public void intakePowerConeRunable(){
    intakePowerSetPoint = Constants.IntakeConstants.kIntakePowerCone;
  }
  public void intakePowerCubeRunable(){
    intakePowerSetPoint = Constants.IntakeConstants.kIntakePowerCube;
  }
  public void runIntake(double power){
    intakeMotor.set(power);
    isRunning = true;
  }
  public void intakeIn(){
    intakeMotor.set(intakePowerSetPoint);
    magicCarpetIn();
    isRunning = true;
    intakeState = IntakeState.IN;
  }
  public void intakeOut(){
    intakeMotor.set(-intakePowerSetPoint);
    magicCarpetOut();
    isRunning = true;
    intakeState = IntakeState.OUT;
  }
  public void intakeOff(){
    intakeMotor.stopMotor();
    if(intakeState == IntakeState.IN){
    } else {magicCarpetOff();}
    isRunning = false;
    intakeState = IntakeState.STOPPED;
  }

  public void runMagicCarpet(double power){
    magicCarpetMotor.set(power);
    isRunning = true;
  }
  public void magicCarpetIn(){
    magicCarpetMotor.set(IntakeConstants.kMagicCarpetPower);
    isRunning = true;
  }
  public void magicCarpetOut(){
    magicCarpetMotor.set(-IntakeConstants.kMagicCarpetPower);
    isRunning = true;
  }
  public void magicCarpetOff(){
    magicCarpetMotor.stopMotor();
    isRunning = false;
  }
  public void intakeExtend(){
    intakeSolenoid.set(true);
  }
  public void intakeRetract(){
    intakeSolenoid.set(false);
  }
  public boolean getIsRunning(){
    return isRunning;
  }
}

