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
  private final double rotKeepOut = 20; // Rotational keepout (degrees) to limit extension
  private final double SHOULDER_LENGTH = 19.25; // inches
  private final double ELBOW_LENGTH = 31.5; // inches
  private final double SAFETY_FACTOR = 4; // inches
  private final double SHOULDER_HEIGHT = 28.5;
  private final double MAX_HEIGHT = 6 * 12 + 6 - SAFETY_FACTOR - SHOULDER_HEIGHT;
  private final double MAX_EXTENSION = 48 - SAFETY_FACTOR;

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
    shoulder1 = new CANSparkMax(Constants.ArmConstants.kShoulder1Port, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.ArmConstants.kShoulder2Port, MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.kElbowPort, MotorType.kBrushless);

    shoulder1.setInverted(Constants.ArmConstants.kShoulder1Inv);
    shoulder2.setInverted(Constants.ArmConstants.kShoulder2Inv);

    shoulder2.follow(shoulder1, true);

    shoulder1.setIdleMode(IdleMode.kBrake);
    shoulder2.setIdleMode(IdleMode.kBrake);
    elbow.setIdleMode(IdleMode.kBrake);
    elbow.setInverted(true);

    elbow.setSmartCurrentLimit(Constants.ArmConstants.Elbow.kCurrentLimit);

    elbowEncoder = elbow.getAbsoluteEncoder(Type.kDutyCycle);
    elbowPIDController = elbow.getPIDController();
    elbowPIDController.setFeedbackDevice(elbowEncoder);
    elbowPIDController.setPositionPIDWrappingEnabled(false);
    elbowEncoder.setInverted(true);
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

    //Not getting valid position during init, so hard coding initial positions.
    elbowTrapController = new ElbowSubsystem(elbowPIDController, Constants.ArmConstants.elbow,
        Math.toRadians(40), elbowEncoder, shoulderEncoder);
    elbowTrapController.disable();

    shoulderTrapController = new ShoulderSubsystem(shoulderPIDController, Constants.ArmConstants.shoulder,
        Math.toRadians(180.0), shoulderEncoder, elbowEncoder);
    shoulderTrapController.disable();

    // Set the PID gains for the elbow motor. These values are set in the Trapezoid
    // Controller above.
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
    // ArmConstants.Shoulder.kMaxOutput);

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

    // //PID tuning. Should be commented out once tuning is complete
    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("ElbowP", 0);
    // double i = SmartDashboard.getNumber("ElbowI", 0);
    // double d = SmartDashboard.getNumber("ElbowD", 0);
    // double ps = SmartDashboard.getNumber("ShoulderP", 0);
    // double is = SmartDashboard.getNumber("ShoulderI", 0);
    // double ds = SmartDashboard.getNumber("ShoulderD", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if((p != elbowPIDController.getP())) { elbowPIDController.setP(p);}
    // if((i != elbowPIDController.getI())) { elbowPIDController.setI(i);}
    // if((d != elbowPIDController.getD())) { elbowPIDController.setD(d);}
    // if((ps != shoulderPIDController.getP())) { shoulderPIDController.setP(ps);}
    // if((is != shoulderPIDController.getI())) { shoulderPIDController.setI(is);}
    // if((ds != shoulderPIDController.getD())) { shoulderPIDController.setD(ds);}

  }

  public void setElbowPosition(double position) {
    // Should calibrate Absolute Encoder on SparkMAX to 180 with elbow straight.
    if (position > ArmConstants.Elbow.kLowerLimit && position < ArmConstants.Elbow.kUpperLimit) {
      elbowTrapController.enable();
      elbowTrapController.setGoal(position);
    } else {
      // Do nothing. Invalid setpoint
    }
  }

  public void setShoulderPosition(double position) {
    // Should calibrate Absolute Encoder on SparkMAX to zero with arm straight down.
    if (position > ArmConstants.Shoulder.kLowerLimit && position < ArmConstants.Shoulder.kUpperLimit) {
      shoulderTrapController.enable();
      shoulderTrapController.setGoal(position);
    } else {
      // Do nothing. Invalid shoulder position
    }
  }

  public double readElbowCurrentPos() {
    return elbowEncoder.getPosition();
  }

  public double getElbowPosition() {
    return elbowPosition;
  }

  public double getElbowPositionDeg() {
    return Math.toDegrees(elbowPosition);
  }

  public double readShoulderCurrentPos() {
    return shoulderEncoder.getPosition();
  }

  public double getShoulderPosition() {
    return shoulderPosition;
  }

  public double getShoulderPositionDeg() {
    return Math.toDegrees(shoulderPosition);
  }

  public void runShoulder(double power) {
    if (!shoulderPowerOk(power, shoulderPosition, elbowPosition)) {
      power = 0;
    }
    shoulderTrapController.disable();
    shoulder1.set(Constants.ArmConstants.Shoulder.kManualScale * power);
    shoulderTrapController.setGoal(shoulderPosition); // Keep trap controller updated with position
  }

  public void runElbow(double power) {
    if (!elbowPowerOk(power, shoulderPosition, elbowPosition)) {
      power = 0;
    }
    elbowTrapController.disable();
    elbow.set(Constants.ArmConstants.Elbow.kManualScale * power);
    elbowTrapController.setGoal(elbowPosition); // Keep trap controller updated with position
  }

  public void holdElbowPosition() {
    elbowTrapController.holdArmPosition();
  }

  public void holdShoulderPosition() {
    shoulderTrapController.holdArmPosition();
  }

  public Command setArmPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> {
      setShoulderPosition(position.shoulderRadians);
      setElbowPosition(position.elbowRadians);
      SmartDashboard.putNumber("Elbow Target", position.elbowDegrees);
      SmartDashboard.putNumber("Shoulder Target", position.shoulderDegrees);
    }, this);
  }

  public boolean elbowIsStraight() {
    return !(getElbowPositionDeg() > 180 + rotKeepOut || getElbowPositionDeg() < 180 - rotKeepOut);
  }

  public boolean shoulderIsMaxExtension() {
    boolean test1 = (getShoulderPositionDeg() > 180 - rotKeepOut) && (getShoulderPositionDeg() < 180 + rotKeepOut);
    boolean test2 = (getShoulderPositionDeg() > 270 - rotKeepOut) && (getShoulderPositionDeg() < 270 + rotKeepOut);
    return test1 || test2;
  }

  public double getArmHeight(double shoulderPos, double elbowPos) {
    return -SHOULDER_LENGTH * Math.cos(shoulderPos) - ELBOW_LENGTH * Math.cos(shoulderPos + elbowPos - Math.PI);
}

public double getArmExtension(double shoulderPos, double elbowPos) {
    return SHOULDER_LENGTH * Math.sin(shoulderPos) + ELBOW_LENGTH * Math.cos(shoulderPos + elbowPos - 3 * Math.PI / 2);
}

public boolean heightDanger(double shoulderPos, double elbowPos) {
    return getArmHeight(shoulderPos, elbowPos) >= MAX_HEIGHT;
}

public boolean extensionDanger(double shoulderPos, double elbowPos) {
    // Checking for negative values here as critical extension is behind robot.
    return getArmExtension(shoulderPos, elbowPos) <= -MAX_EXTENSION;
}

public double heightSensShoulder(double shoulderPos, double elbowPos) {
    return SHOULDER_LENGTH * Math.sin(shoulderPos) - ELBOW_LENGTH * Math.sin(shoulderPos + elbowPos);
}

public boolean heightSensShoulderIsPositive(double shoulderPos, double elbowPos) {
    return heightSensShoulder(shoulderPos, elbowPos) >= 0;
}

public double heightSensElbow(double shoulderPos, double elbowPos) {
    return -ELBOW_LENGTH * Math.sin(shoulderPos + elbowPos);
}

public boolean heightSensElowIsPositive(double shoulderPos, double elbowPos) {
    return heightSensElbow(shoulderPos, elbowPos) >= 0;
}

public double extensionSensShoulder(double shoulderPos, double elbowPos) {
    return SHOULDER_LENGTH * Math.cos(shoulderPos) - ELBOW_LENGTH * Math.cos(shoulderPos + elbowPos);
}

public boolean extensionSensShoulderIsPositive(double shoulderPos, double elbowPos) {
    // Extension critical value is negative. Need to check of decreasing value in
    // critical regions
    return extensionSensShoulder(shoulderPos, elbowPos) >= 0;
}

public double extensionSensElbow(double shoulderPos, double elbowPos) {
    return -ELBOW_LENGTH * Math.cos(shoulderPos + elbowPos);
}

public boolean extensionSensElbowIsPositive(double shoulderPos, double elbowPos) {
    // Extension critical value is negative. Need to check of decreasing value in
    // critical regions
    return extensionSensElbow(shoulderPos, elbowPos) >= 0;
}

public boolean elbowPowerOk(double elbowPower, double shoulderPos, double elbowPos) {
    boolean powerOk;

    if (heightDanger(shoulderPos, elbowPos)) {
        powerOk = (Math.signum(elbowPower) * Math.signum(heightSensElbow(shoulderPos, elbowPos))) <= 0;
    } else if (extensionDanger(shoulderPos, elbowPos)) {
        powerOk = (Math.signum(elbowPower) * Math.signum(extensionSensElbow(shoulderPos, elbowPos))) >= 0;
    } else {
        powerOk = true;
    }

    return powerOk;
}

public boolean shoulderPowerOk(double shoulderPower, double shoulderPos, double elbowPos) {
    boolean powerOk;

    if (heightDanger(shoulderPos, elbowPos)) {
        powerOk = (Math.signum(shoulderPower) * Math.signum(heightSensShoulder(shoulderPos, elbowPos))) <= 0;
    } else if (extensionDanger(shoulderPos, elbowPos)) {
        powerOk = (Math.signum(shoulderPower) * Math.signum(extensionSensShoulder(shoulderPos, elbowPos))) >= 0;
    } else {
        powerOk = true;
    }

    return powerOk;
}

}
