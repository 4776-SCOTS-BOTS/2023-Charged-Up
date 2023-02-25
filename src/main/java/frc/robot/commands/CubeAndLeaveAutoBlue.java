// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CubeAndLeaveAutoBlue extends SequentialCommandGroup {
  /** Creates a new CubeAndLeaveAuto. */
  public CubeAndLeaveAutoBlue(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake) {
    // 137" = 3.5m

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics).setReversed(false);

    Trajectory driveToLineTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Drive Forward
        List.of(new Translation2d(1.0, -0.5)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3.5, -0.5, new Rotation2d(Math.toRadians(0))),
        config);

    var thetaController = new ProfiledPIDController(
        2, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand driveToLine = new SwerveControllerCommand(
        driveToLineTrajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(driveToLineTrajectory.getInitialPose());

    addCommands(
        // Extend arm
        new InstantCommand(() -> {
          arm.setArmPositionCommand(Constants.ArmConstants.CUBE_HIGH_POSITION);
        }, arm),

        // Wait and release
        new WaitCommand(2),
        new InstantCommand(gripper::openGripper, gripper),

        // Pack the arm
        new InstantCommand(() -> {
          arm.setArmPositionCommand(Constants.ArmConstants.SAFE_POSITION);
        }, arm),

        // Drive to the ball
        driveToLine.andThen(() -> drive.drive(0, 0, 0, false)));
  }
}
