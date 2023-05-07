// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.customClass.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceFirstCone extends SequentialCommandGroup {
  /** Creates a new PlaceFirstCone. */
  DataLog log = DataLogManager.getLog();
  StringLogEntry statusLog = new StringLogEntry(log, "/my/status");
  StringLogEntry poseLog = new StringLogEntry(log, "/my/Pose");

  public PlaceFirstCone(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake, Pose2d startPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Reset odometry to the starting pose of the trajectory.
        new InstantCommand(() -> statusLog.append("Starting Place")),
        new InstantCommand(() -> poseLog.append(drive.poseEstimator.getFormattedPose())),
        
        new InstantCommand(() -> drive.resetOdometry(startPose)),
        new InstantCommand(() -> drive.poseEstimator.setCurrentPose(startPose)),

        new InstantCommand(() -> statusLog.append("Reset Poses")),
        new InstantCommand(() -> poseLog.append(drive.poseEstimator.getFormattedPose())),
        

        //Set initial arm position
        //new InstantCommand(intake::intakeExtend),
        // new WaitCommand(0.5), 
        //arm.setArmPositionCommand(Constants.ArmConstants.READY_POSITION_CONE),
        //new MoveShoulderThenElbow(arm, Constants.ArmConstants.READY_POSITION_CUBE),

        // Drive against wall and ready arm
        new ParallelCommandGroup(
            //new DriveToWall(drive, 0.5),
            new MoveShoulderThenElbow(arm, ArmConstants.HIGH_POSITION_FINAL)),
            new WaitCommand(1.0),
        
        // Stop drive, extend and drop
        new InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
        new InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
        //new MultiStepArm(arm, ArmConstants.HIGH_POSITION, ArmConstants.HIGH_POSITION),
        arm.setArmPositionCommand(new ArmPosition(ArmConstants.HIGH_POSITION_FINAL.elbowDegrees, 
        ArmConstants.HIGH_POSITION_FINAL.shoulderDegrees+7)),
        new WaitCommand(0.25),
        //new InstantCommand(gripper::extendKicker),
        //new InstantCommand(intake::intakeRetract),
        new InstantCommand(gripper::openGripper, gripper),
        new WaitCommand(0.2),
        new InstantCommand(intake::intakeExtend),
        //new InstantCommand(gripper::retractKicker),

        new InstantCommand(() -> statusLog.append("Leaving Place")),
        new InstantCommand(() -> poseLog.append(drive.poseEstimator.getFormattedPose()))


    );
  }
}
