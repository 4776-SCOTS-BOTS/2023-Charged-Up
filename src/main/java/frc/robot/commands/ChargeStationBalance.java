// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChargeStationBalance extends CommandBase {
  /** Creates a new ChargeStationBalance. */
  private DriveSubsystem drive;
  private double startTime, holdStart, timeout, holdTime;


  public ChargeStationBalance(DriveSubsystem drive, double holdTime, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    this.timeout = timeout;
    this.holdTime = holdTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = drive.getPitch();
    // SmartDashboard.putNumber("Pitch", pitch);

    //drive.drive(0.5, 0, 0, false);
    if(pitch > 5){
      holdStart = 1000; // Arbitrarily large value
      drive.drive(0.45, 0, 0, false);
    } else if (pitch < -5){
      holdStart = 1000; // Arbitrarily large value1
      drive.drive(-0.45, 0, 0, false);
    } else if (holdStart == 1000){
      holdStart = Timer.getFPGATimestamp();
      drive.drive(0, 0, 0, false);
    } else {
      drive.setXModuleState();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setXModuleState();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - holdStart) > holdTime) || ((Timer.getFPGATimestamp() - startTime) > timeout);
  }
}
