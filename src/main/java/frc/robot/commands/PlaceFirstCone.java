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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceFirstCone extends SequentialCommandGroup {
  /** Creates a new PlaceFirstCone. */
  public PlaceFirstCone(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake, Pose2d startPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Reset odometry to the starting pose of the trajectory.
        new InstantCommand(() -> drive.resetOdometry(startPose)),
        new InstantCommand(() -> drive.poseEstimator.setCurrentPose(startPose)),

        //Set initial arm position
        new InstantCommand(intake::intakeExtend),
        new WaitCommand(0.5), 
        arm.setArmPositionCommand(Constants.ArmConstants.READY_POSITION_CONE),

        // Drive against wall and ready arm
        new DriveToWall(drive, 0.25),

        
        // Stop drive
        new InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
        // new MultiStepArm(arm, Constants.ArmConstants.READY_POSITION_CONE,
        //     Constants.ArmConstants.READY_POSITION_CONE),
        new WaitCommand(1.0),
        new InstantCommand(intake::intakeRetract),

        // Extend arm and release
        new MultiStepArm(arm, Constants.ArmConstants.HIGH_POSITION_START,
            Constants.ArmConstants.HIGH_POSITION),
            arm.setArmPositionCommand(Constants.ArmConstants.HIGH_POSITION_FINAL),
        new WaitCommand(0.5),
        new InstantCommand(gripper::openGripper, gripper),

        // Pack the arm
        arm.setArmPositionCommand(Constants.ArmConstants.READY_POSITION_CONE),
        new WaitCommand(0.1),
        arm.setArmPositionCommand(Constants.ArmConstants.SAFE_POSITION)
    );
  }
}
