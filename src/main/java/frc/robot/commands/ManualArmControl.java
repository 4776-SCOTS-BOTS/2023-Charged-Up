// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;

public class ManualArmControl extends CommandBase {
  private Arm arm;
  private double elbowPos, shoulderPos;
  private double newElbowPos, newShoulderPos;

  /** Creates a new ManualArmControl. */
  public ManualArmControl(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbowPos = arm.readElbowCurrentPos();
    shoulderPos = arm.readShoulderCurrentPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elbowPos = arm.readElbowCurrentPos();
    shoulderPos = arm.readShoulderCurrentPos();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
