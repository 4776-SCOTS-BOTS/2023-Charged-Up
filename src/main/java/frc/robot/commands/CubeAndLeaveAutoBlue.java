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

        TrajectoryConfig configReversed = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics).setReversed(true);

    Trajectory driveToLineTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Drive Forward
        List.of(new Translation2d(1.0, -0.5)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3.7, -0.5, new Rotation2d(Math.toRadians(0))),
        config);

        Trajectory driveBackTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(3.7, -0.5, new Rotation2d(Math.toRadians(0))),
        // Drive Forward
        List.of(new Translation2d(3.0, -0.5)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2.5, -0.5, new Rotation2d(Math.toRadians(0))),
        configReversed);

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

        SwerveControllerCommand driveBack = new SwerveControllerCommand(
        driveBackTrajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    addCommands(
        // Reset odometry to the starting pose of the trajectory.
        new InstantCommand(() -> drive.resetOdometry(driveToLineTrajectory.getInitialPose())),

        // Extend arm
        arm.setArmPositionCommand(Constants.ArmConstants.CUBE_HIGH_POSITION),

        // Wait and release
        new WaitCommand(2),
        new InstantCommand(gripper::openGripper, gripper),
        new WaitCommand(2),

        // Pack the arm
        arm.setArmPositionCommand(Constants.ArmConstants.SAFE_POSITION),

        // Drive over line
        driveToLine,

        //Drive back
        driveBack.andThen(() -> drive.drive(0, 0, 0, false)));
  }
}
