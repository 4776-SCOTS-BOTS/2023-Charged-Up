// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.customClass.ArmPosition;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
  private final ShoulderSubsystem shoulderTrapController;

  public double elbowPosition;
  public double shoulderPosition;

  private boolean elbowInManual = false;
  private boolean shoulderInManual = false;

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
    shoulderEncoder.setZeroOffset(Constants.ArmConstants.shoulder.kOffset);

    elbowTrapController = new ElbowSubsystem(elbowPIDController, Constants.ArmConstants.elbow,
        readElbowCurrentPos(), elbowEncoder, shoulderEncoder);
    elbowTrapController.disable();

    shoulderTrapController = new ShoulderSubsystem(shoulderPIDController, Constants.ArmConstants.shoulder,
        readShoulderCurrentPos(), shoulderEncoder, elbowEncoder);
    shoulderTrapController.disable();

    // Set the PID gains for the elbow motor.  These values are set in the Trapezoid Controller above.
    // elbowPIDController.setP(ArmConstants.Elbow.kP);
    // elbowPIDController.setI(ArmConstants.Elbow.kI);
    // elbowPIDController.setD(ArmConstants.Elbow.kD);
    // elbowPIDController.setFF(ArmConstants.Elbow.kFF);
    // elbowPIDController.setOutputRange(ArmConstants.Elbow.kMinOutput,
    // ArmConstants.Elbow.kMaxOutput);

    // Set the PID gains for the shoulder motor.
    // shoulderPIDController.setP(ArmConstants.Shoulder.kP);
    // shoulderPIDController.setI(ArmConstants.Shoulder.kI);
    // shoulderPIDController.setD(ArmConstants.Shoulder.kD);
    // shoulderPIDController.setFF(ArmConstants.Shoulder.kFF);
    // shoulderPIDController.setOutputRange(ArmConstants.Shoulder.kMinOutput,
    //     ArmConstants.Shoulder.kMaxOutput);

    Shuffleboard.getTab("Arm").addNumber("Elbow", this::getElbowPositionDeg);
    Shuffleboard.getTab("Arm").addNumber("Shoulder", this::getShoulderPositionDeg);

    // display PID coefficients on SmartDashboard
    // Comment out once tuning is done
    // SmartDashboard.putNumber("ElbowP", elbowPIDController.getP());
    // SmartDashboard.putNumber("ElbowI", elbowPIDController.getI());
    // SmartDashboard.putNumber("ElbowD", elbowPIDController.getD());
    // SmartDashboard.putNumber("ShoulderP", elbowPIDController.getP());
    // SmartDashboard.putNumber("ShoulderI", elbowPIDController.getI());
    // SmartDashboard.putNumber("ShoulderD", elbowPIDController.getD());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elbowPosition = readElbowCurrentPos();
    shoulderPosition = readShoulderCurrentPos();

    elbowTrapController.setOffsetAngleRads(shoulderPosition);
    shoulderTrapController.setOffsetAngleRads(elbowPosition);

    // //PID tuning.  Should be commented out once tuning is complete
    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("ElbowP", 0);
    // double i = SmartDashboard.getNumber("ElbowI", 0);
    // double d = SmartDashboard.getNumber("ElbowD", 0);
    // double ps = SmartDashboard.getNumber("ShoulderP", 0);
    // double is = SmartDashboard.getNumber("ShoulderI", 0);
    // double ds = SmartDashboard.getNumber("ShoulderD", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != elbowPIDController.getP())) { elbowPIDController.setP(p);}
    // if((i != elbowPIDController.getI())) { elbowPIDController.setI(i);}
    // if((d != elbowPIDController.getD())) { elbowPIDController.setD(d);}
    // if((ps != shoulderPIDController.getP())) { shoulderPIDController.setP(ps);}
    // if((is != shoulderPIDController.getI())) { shoulderPIDController.setI(is);}
    // if((ds != shoulderPIDController.getD())) { shoulderPIDController.setD(ds);}

  }

  public void setElbowPosition(double position){
    //Should calibrate Absolute Encoder on SparkMAX to 180 with elbow straight.
    if(position > ArmConstants.Elbow.kLowerLimit && position < ArmConstants.Elbow.kUpperLimit){
      elbowTrapController.enable();
      elbowTrapController.setGoal(position);
    } else {
      //Do nothing.  Invalid setpoint
    }
  }

  public void setShoulderPosition(double position){
    //Should calibrate Absolute Encoder on SparkMAX to zero with arm straight down.
    if(position > ArmConstants.Shoulder.kLowerLimit && position < ArmConstants.Shoulder.kUpperLimit){
      shoulderTrapController.enable();
      shoulderTrapController.setGoal(position);
    } else {
      //Do nothing.  Invalid shoulder position
    }
  }

  public double readElbowCurrentPos(){
    return elbowEncoder.getPosition();
  }

  public double getElbowPosition(){
    return elbowPosition;
  }
  
  public double getElbowPositionDeg(){
    return Math.toDegrees(elbowPosition);
  }

  public double readShoulderCurrentPos(){
    return shoulderEncoder.getPosition();
  }

  public double getShoulderPosition(){
    return shoulderPosition;
  }

  public double getShoulderPositionDeg(){
    return Math.toDegrees(shoulderPosition);
  }

  public void runShoulder(double power){
    shoulderTrapController.disable();
    shoulder1.set(Constants.ArmConstants.Shoulder.kManualScale * power);
    shoulderTrapController.setGoal(shoulderPosition); //Keep trap controller updated with position
  }

  public void runElbow(double power){
    elbowTrapController.disable();
    elbow.set(Constants.ArmConstants.Elbow.kManualScale * power);
    elbowTrapController.setGoal(elbowPosition); //Keep trap controller updated with position
  }

  public void holdElbowPosition(){
    elbowTrapController.holdArmPosition();
  }

  public void holdShoulderPosition(){
    shoulderTrapController.holdArmPosition();
  }

  public Command setArmPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> {
      setElbowPosition(position.elbowRadians);
      setShoulderPosition(position.shoulderRadians);
    }, this);
  }

  
}
