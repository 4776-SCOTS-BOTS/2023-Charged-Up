// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import java.io.Serial;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.customClass.CRGB;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.customClass.CRGB;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax magicCarpetMotor;
  private Solenoid intakeSolenoid;
  private Servo tipperServoLeft; 
  private Servo tipperServoRight; 
  private boolean isRunning = false;
  private double intakePowerSetPoint = Constants.IntakeConstants.kIntakePowerCone;

  private LinearFilter intakeCurrentFilter  = LinearFilter.movingAverage(5);
  private double filteredCurrent, rawCurrent;



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
    magicCarpetMotor.setIdleMode(IdleMode.kCoast);
    magicCarpetMotor.setSmartCurrentLimit(15);

    intakeSolenoid = new Solenoid(PneumaticsConstants.phCanID, PneumaticsModuleType.REVPH,PneumaticsConstants.intakeSolenoidPort);
    tipperServoLeft = new Servo(Constants.IntakeConstants.tipperServoPinLeft);
    tipperServoRight = new Servo(Constants.IntakeConstants.tipperServoPinRight); 

    Shuffleboard.getTab("Arm").addNumber("RawCurrent", () -> { return rawCurrent;} );
    Shuffleboard.getTab("Arm").addNumber("FilteredCurrent", () -> { return filteredCurrent;} );
  }

  @Override
  public void periodic(){
    rawCurrent = getIntakeCurrent();
    filteredCurrent = intakeCurrentFilter.calculate(rawCurrent);
  }

  public void setStallLow(){
    intakeMotor.setSmartCurrentLimit(45);
  }

  public void setStallNormal(){
    intakeMotor.setSmartCurrentLimit(45);
  }
  
  public double getIntakeCurrent(){
    return intakeMotor.getOutputCurrent();
  }

  public double getFilteredCurrent(){
    return filteredCurrent;
  }

  public void setIntakePowerCone(){
    intakePowerSetPoint = Constants.IntakeConstants.kIntakePowerCone;
  }
  public void setIntakePowerCube(){
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
  public void magicCarpetFast(){
    magicCarpetMotor.set(-1);
    isRunning = true; 
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
  public void tipperUse(){
    tipperServoLeft.set(Constants.IntakeConstants.tipperUsePosLeft);
    tipperServoRight.set(Constants.IntakeConstants.tipperUsePostRight);
  }
  public void tipperSafe (){
    tipperServoLeft.set(Constants.IntakeConstants.tipperSafePosLeft);
    tipperServoRight.set(Constants.IntakeConstants.tipperSafePosRight);
  }
}

