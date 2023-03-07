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
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
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

public class BlueRightConeCube extends SequentialCommandGroup {
  /** Creates a new CubeAndLeaveAuto. */
  public BlueRightConeCube(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake) {
    Pose2d startPose = new Pose2d(1.905, 1.626, new Rotation2d(0));
    Pose2d pickupPose = new Pose2d(7.14, 0.91, new Rotation2d(Math.toRadians(0)));

    // Create config for trajectory
    RectangularRegionConstraint bumpConstraint = new RectangularRegionConstraint(
        new Translation2d(3.295, 1.524),
        new Translation2d(4.46, 0),
        new SwerveDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 0.5));

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics).setReversed(false)
        .addConstraint(bumpConstraint);

    Trajectory driveToCubeTraj = TrajectoryGenerator.generateTrajectory(
        // Start position
        startPose,
        // Drive to cube
        List.of(new Translation2d(2.1, 0.914),
            new Translation2d(3.86, 0.762)),
        // End end at the cube, facing forward
        new Pose2d(7.14, 0.91, new Rotation2d(Math.toRadians(0))),
        config);

    Trajectory driveToPlaceTraj = TrajectoryGenerator.generateTrajectory(
        // Start position
        pickupPose,
        // Drive to cube
        List.of(new Translation2d(3.86, 0.762),
            new Translation2d(2.1, 0.914)),
        // End end at the cube, facing forward
        new Pose2d(1.905, 1.067, new Rotation2d(Math.toRadians(0))),
        config);

    var thetaController = new ProfiledPIDController(
        2, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand driveToCube = new SwerveControllerCommand(
        driveToCubeTraj,
        drive.poseEstimator::getCurrentPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    SwerveControllerCommand driveToScore = new SwerveControllerCommand(
        driveToPlaceTraj,
        drive.poseEstimator::getCurrentPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    addCommands(
        // Reset odometry to the starting pose of the trajectory.
        new InstantCommand(() -> drive.resetOdometry(startPose)),
        new InstantCommand(() -> drive.poseEstimator.setCurrentPose(startPose)),

        // Drive against wall and ready arm
        new ParallelCommandGroup(
            arm.setArmPositionCommand(Constants.ArmConstants.READY_POSITION3),
            new InstantCommand(() -> drive.drive(-0.2, 0, 0, false)),
            new WaitCommand(1)),

        // Stop drive and let arm finish
        new InstantCommand(() -> drive.drive(0, 0, 0, false)),
        new WaitCommand(1),

        // Extend arm and release
        new MultiStepArm(arm, Constants.ArmConstants.HIGH_POSITION1,
            Constants.ArmConstants.HIGH_POSITION1),
        new InstantCommand(gripper::openGripper, gripper),

        // Pack the arm
        arm.setArmPositionCommand(Constants.ArmConstants.SAFE_POSITION),

        // Drive over line
        new ParallelCommandGroup(
            driveToCube.andThen(() -> drive.drive(0, 0, 0, false)),
            new WaitCommand(1)
                .andThen(new InstantCommand(intake::intakeExtend))
                .andThen(new InstantCommand(intake::intakeIn))),
        new InstantCommand(intake::intakeOff),
        new InstantCommand(intake::intakeOff),

        arm.setArmPositionCommand(Constants.ArmConstants.PICKUP_POSITION),
        new WaitCommand(2),
        new InstantCommand(gripper::closeGripper, gripper),
        arm.setArmPositionCommand(Constants.ArmConstants.SAFE_POSITION),

        // Drive back
        new ParallelCommandGroup(
            new InstantCommand(intake::intakeRetract),
            arm.setArmPositionCommand(Constants.ArmConstants.READY_POSITION1),
            driveToScore.andThen(new InstantCommand(() -> drive.drive(-0.2, 0, 0, false)))),

        new MultiStepArm(arm, Constants.ArmConstants.HIGH_POSITION1,
            Constants.ArmConstants.HIGH_POSITION1),
        new InstantCommand(() -> drive.drive(0, 0, 0, false)),
        new InstantCommand(gripper::openGripper, gripper)

    );

  }
}
