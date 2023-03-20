// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
import frc.robot.customClass.ArmPosition;

public class MultiStepArm extends CommandBase {
  /** Creates a new MultiStepArm. */
  private Arm m_Arm;
  private ArmPosition position1, position2;
  private boolean inStage1 = false;
  private double startTime, timeout;

  public MultiStepArm(Arm arm, ArmPosition position1, ArmPosition position2, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = arm;
    addRequirements(m_Arm);

    this.position1 = position1;
    this.position2 = position2;
    this.timeout = timeout;
  }

  public MultiStepArm(Arm arm, ArmPosition position1, ArmPosition position2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(arm, position1, position2, 2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.setShoulderPosition(position1.shoulderRadians);
    m_Arm.setElbowPosition(position1.elbowRadians);
    inStage1 = true;
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (inStage1) {
      double shoulderPos = m_Arm.getShoulderPositionDeg();
      double elbowPos = m_Arm.getElbowPositionDeg();
      if (Math.abs(shoulderPos - position1.shoulderDegrees) < 10 && Math.abs(elbowPos - position1.elbowDegrees) < 10) {
        m_Arm.setShoulderPosition(position2.shoulderRadians);
        m_Arm.setElbowPosition(position2.elbowRadians);
        inStage1 = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setShoulderPosition(position2.shoulderRadians);
    m_Arm.setElbowPosition(position2.elbowRadians);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double shoulderPos = m_Arm.getShoulderPositionDeg();
    return (Math.abs(shoulderPos - position1.shoulderDegrees) < 5) || (Timer.getFPGATimestamp() - startTime > timeout);
  }
}
