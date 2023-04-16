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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BlueLeftConeTest extends SequentialCommandGroup {
    /** Creates a new CubeAndLeaveAuto. */
    DataLog log = DataLogManager.getLog();
    StringLogEntry statusLog = new StringLogEntry(log, "/my/status");

    public BlueLeftConeTest(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake) {
        Pose2d startPose = new Pose2d(0, Units.inchesToMeters(0), new Rotation2d(0));
        Pose2d pickupPose = new Pose2d(Units.feetToMeters(14), Units.inchesToMeters(0),
                new Rotation2d(Math.toRadians(0)));

        // Create config for trajectory
        // RectangularRegionConstraint bumpConstraint = new
        // RectangularRegionConstraint(new Translation2d(3.295, 1.524),
        // new Translation2d(4.46, 0),
        // new SwerveDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 0.25));

        RectangularRegionConstraint bumpConstraint = new RectangularRegionConstraint(new Translation2d(3.295, 0),
                new Translation2d(4.46, 1.524),
                new MaxVelocityConstraint(1.0));

        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setEndVelocity(2.5)
                .setReversed(false);

        Trajectory driveToCubeTraj = TrajectoryGenerator.generateTrajectory(
                // Start position
                startPose,
                // Drive to cube
                List.of(new Translation2d(0.75, Units.inchesToMeters(0)),
                        new Translation2d(1.5, Units.inchesToMeters(0))),
                // End end at the cube, facing forward
                new Pose2d(pickupPose.getX() - 1.5, pickupPose.getY(), pickupPose.getRotation()),
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
                new InstantCommand(() -> statusLog.append("Starting Blue Left Cone")),

                new InstantCommand(() -> {
                    Constants.ConfigConstants.alliance = Alliance.Blue;
                }),
                // Reset odometry to the starting pose of the trajectory.
                new InstantCommand(() -> drive.resetOdometry(startPose)),
                new InstantCommand(() -> drive.poseEstimator.setCurrentPose(startPose)),
                // new PlaceFirstCone(drive, arm, gripper, intake, startPose),
                // new MoveElbowThenShoulder(arm, ArmConstants.SAFE_POSITION),

                // Drive over line

                driveToCube,
        new ChaseCube(drive, pickupPose, 2.5, 5)
        );

    }
}