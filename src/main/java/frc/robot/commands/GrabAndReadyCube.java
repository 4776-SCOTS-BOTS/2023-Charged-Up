// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabAndReadyCube extends SequentialCommandGroup {
  /** Creates a new GrabAndReadyCube. */
  private Arm arm;
  private Intake intake;
  private Gripper gripper;

  public GrabAndReadyCube(Arm arm, Intake intake, Gripper gripper) {
    this.arm = arm;
    this.intake = intake;
    this.gripper = gripper;

    addCommands(
        new WaitCommand(1.0),
        new InstantCommand(gripper::openGripper),
        new MultiStepArm(arm, Constants.ArmConstants.PICKUP_POSITION_CUBE, Constants.ArmConstants.PICKUP_POSITION_CUBE),
        new InstantCommand(gripper::closeGripper),
        new InstantCommand(intake::intakeExtend),
        arm.setArmPositionCommand(Constants.ArmConstants.READY_POSITION_CUBE));
  }
}
