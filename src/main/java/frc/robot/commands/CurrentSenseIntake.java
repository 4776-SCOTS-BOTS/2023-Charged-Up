// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.customClass.ArmJointConstants;
import frc.robot.customClass.ArmPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;

public class CurrentSenseIntake extends CommandBase {
  /** Creates a new CurrentSenseIntake. */
  private Intake intake;
  private Arm arm;
  private double startTime, stallTime;
  private boolean stallStarted;
  private double stallFinishedTime;
  private boolean isFinished;
  private boolean finalFinished;

  private final double START_DELAY = 0.25; //seconds
  private final double STALL_DELAY = 0.18;
  private final double STALL_CURRENT = 30;//AMPS
  private final double FINAL_BLIP = 0.5; //seconds

  public CurrentSenseIntake(Intake intake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(arm);

    this.intake = intake;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setShoulderPosition(ArmConstants.SAFE_POSITION.shoulderRadians);
    arm.setElbowPosition(Math.toRadians(45));
    intake.intakeIn();
    intake.runIntake(0.7*Constants.IntakeConstants.kIntakePowerCone);
    intake.magicCarpetOff();
    intake.setStallLow();
    startTime = Timer.getFPGATimestamp();
    stallStarted = false;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp()-startTime > START_DELAY){
      if(!stallStarted && intake.getFilteredCurrent() >= STALL_CURRENT){
        stallStarted = true;
        stallTime = Timer.getFPGATimestamp();
      } else if (stallStarted && (Timer.getFPGATimestamp() - stallTime >= STALL_DELAY)){
        intake.intakeOff();
        isFinished = true;
        stallFinishedTime = Timer.getFPGATimestamp();
      }
    }
    // if(isFinished){
    //   intake.runIntake(0.25);
    //   if(Timer.getFPGATimestamp()-stallFinishedTime > FINAL_BLIP){
    //     finalFinished = true;
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
    intake.setStallNormal();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
