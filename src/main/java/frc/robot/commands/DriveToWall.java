// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToWall extends CommandBase {
  /** Creates a new ChargeStationBalance. */
  private DriveSubsystem drive;
  private double startTime, holdStart, timeout, holdTime;


  public DriveToWall(DriveSubsystem drive, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    this.timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drive.drive(-0.2, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) >= timeout;
  }
}
