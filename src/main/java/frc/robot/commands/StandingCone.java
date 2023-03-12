// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Gripper;

public class StandingCone extends CommandBase {
  /** Creates a new StandingCone. */
  private double carpetTime = 0.4;
  private double startTime;
  private Arm arm;
  private Intake intake;
  private Gripper gripper;


  public StandingCone(Arm arm, Gripper gripper, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    this.gripper = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    intake.magicCarpetOut();
    arm.setArmPositionCommand(Constants.ArmConstants.PICKUP_STANDING_CONE);
    gripper.openGripper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intake.magicCarpetOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime > carpetTime);
  }
}
