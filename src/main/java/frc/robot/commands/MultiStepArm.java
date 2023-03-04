// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

  public MultiStepArm(Arm arm, ArmPosition position1, ArmPosition position2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Arm = arm;
    addRequirements(m_Arm);

    this.position1 = position1;
    this.position2 = position2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.setArmPositionCommand(position1);
    inStage1 = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (inStage1) {
      double shoulderPos = m_Arm.getElbowPositionDeg();
      if (Math.abs(shoulderPos - position1.shoulderDegrees) < 10) {
        m_Arm.setArmPositionCommand(position2);
        inStage1 = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double shoulderPos = m_Arm.getElbowPositionDeg();
    return Math.abs(shoulderPos - position1.shoulderDegrees) < 10;
  }
}
