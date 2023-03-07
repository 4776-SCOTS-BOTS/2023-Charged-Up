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
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
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

public class RedMidConePark extends SequentialCommandGroup {
    /** Creates a new CubeAndLeaveAuto. */
    public RedMidConePark(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake) {
        Pose2d startPose = new Pose2d(1.905, Units.inchesToMeters(229.5), new Rotation2d(0));
        Pose2d pickupPose = new Pose2d(7.14, Units.inchesToMeters(231), new Rotation2d(Math.toRadians(0)));

        // Create config for trajectory
        // RectangularRegionConstraint bumpConstraint = new
        // RectangularRegionConstraint(new Translation2d(3.295, 1.524),
        // new Translation2d(4.46, 0),
        // new SwerveDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 0.25));

        RectangularRegionConstraint rampConstraint = new RectangularRegionConstraint(new Translation2d(2.4, Units.inchesToMeters(154)),
                new Translation2d(3.5, Units.inchesToMeters(256)),
                new MaxVelocityConstraint(0.5));

        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics).setReversed(false)
                .addConstraint(rampConstraint);

        TrajectoryConfig configBalance = new TrajectoryConfig(
                0.5,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics).setReversed(false);

        Trajectory driveToCubeTraj = TrajectoryGenerator.generateTrajectory(
                // Start position
                startPose,
                // Drive to cube
                List.of(new Translation2d(2.1, Units.inchesToMeters(207.5)),
                        new Translation2d(Units.inchesToMeters(96.75 + 54), Units.inchesToMeters(207.5)),
                        new Translation2d(Units.inchesToMeters(96.75 + 54 + 36), Units.inchesToMeters(207.5))),
                // End end at the cube, facing forward
                pickupPose,
                config);

        Trajectory driveToBalanceTraj = TrajectoryGenerator.generateTrajectory(
                // Start position
                pickupPose,
                // Drive to Center of Charging Station
                List.of(new Translation2d(Units.inchesToMeters(96.75 + 54 + 36), Units.inchesToMeters(207.5))),
                // End end at the cube, facing forward
                new Pose2d(Units.inchesToMeters(96.75 + 54), Units.inchesToMeters(207.5),
                        new Rotation2d(Math.toRadians(0))),
                configBalance);

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

        SwerveControllerCommand driveToBalance = new SwerveControllerCommand(
                driveToBalanceTraj,
                drive.poseEstimator::getCurrentPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                thetaController,
                drive::setModuleStates,
                drive);

        addCommands(
                new PlaceFirstCone(drive, arm, gripper, intake, startPose),

                // Drive over line
                new ParallelCommandGroup(
                        driveToCube.andThen(() -> drive.drive(0, 0, 0, false)),
                        new WaitCommand(2)
                                .andThen(new InstantCommand(intake::intakeExtend))
                                .andThen(new InstantCommand(intake::intakeIn))),
                new InstantCommand(intake::intakeOff),
                new InstantCommand(intake::intakeOff),

                // Drive back
                new ParallelCommandGroup(
                        new InstantCommand(intake::intakeRetract),
                        driveToBalance.andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false)))),

                new InstantCommand(drive::setXModuleState)

        );

    }
}
