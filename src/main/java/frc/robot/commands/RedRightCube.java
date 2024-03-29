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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RedRightCube extends SequentialCommandGroup {
    /** Creates a new CubeAndLeaveAuto. */
    public RedRightCube(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake) {
        // Field width = 315.5in
        Pose2d startPose = new Pose2d(1.905, Units.inchesToMeters(141.5), new Rotation2d(0));
        Pose2d pickupPose = new Pose2d(7.14, Units.inchesToMeters(146), new Rotation2d(Math.toRadians(0)));

        // Create config for trajectory
        // RectangularRegionConstraint bumpConstraint = new
        // RectangularRegionConstraint(new Translation2d(3.295, 1.524),
        // new Translation2d(4.46, 0),
        // new SwerveDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 0.25));

        RectangularRegionConstraint bumpConstraint = new RectangularRegionConstraint(
                new Translation2d(3.295, Units.inchesToMeters(256)),
                new Translation2d(4.46, Units.inchesToMeters(315.5)),
                new MaxVelocityConstraint(0.5));

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
                List.of(new Translation2d(2.1, Units.inchesToMeters(140)),
                        new Translation2d(3.86, Units.inchesToMeters(140))),
                // End end at the cube, facing forward
                pickupPose,
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

        addCommands(
                new InstantCommand(()->{Constants.ConfigConstants.alliance = Alliance.Red;}),         
                new PlaceFirstCube(drive, arm, gripper, intake, startPose),

                // Drive over line
                new ParallelCommandGroup(
                        driveToCube.andThen(() -> drive.drive(0, 0, 0, false)),
                        new WaitCommand(1)
                                .andThen(new InstantCommand(intake::intakeExtend))
                                .andThen(new InstantCommand(intake::intakeIn))),
                new WaitCommand(2),
                new InstantCommand(intake::intakeOff),
                new InstantCommand(intake::intakeOff),
                new InstantCommand(intake::intakeRetract));

    }
}
