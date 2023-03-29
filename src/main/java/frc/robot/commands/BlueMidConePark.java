// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import javax.xml.crypto.Data;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.PlaceFirstCone;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BlueMidConePark extends SequentialCommandGroup {
  /** Creates a new CubeAndLeaveAuto. */
  DataLog log = DataLogManager.getLog();
  StringLogEntry statusLog = new StringLogEntry(log, "/my/status");

  public BlueMidConePark(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake) {
    Pose2d startPose = new Pose2d(Units.inchesToMeters(75), Units.inchesToMeters(86.0), new Rotation2d(0));
    //Pose2d pickupPose = new Pose2d(7.1, Units.inchesToMeters(108), new Rotation2d(Math.toRadians(0)));
    //Pose2d pickupPoseFlipped = new Pose2d(7.1, Units.inchesToMeters(108), new Rotation2d(Math.toRadians(180)));
    Pose2d balancePose = new Pose2d(Units.inchesToMeters(140), Units.inchesToMeters(108), new Rotation2d(Math.toRadians(0)));

    // Create config for trajectory
    // RectangularRegionConstraint bumpConstraint = new
    // RectangularRegionConstraint(new Translation2d(3.295, 1.524),
    // new Translation2d(4.46, 0),
    // new SwerveDriveKinematicsConstraint(DriveConstants.kDriveKinematics, 0.25));

    // RectangularRegionConstraint rampConstraint = new RectangularRegionConstraint(new Translation2d(2.6, 1.524),
    //     new Translation2d(3.5, 4),
    //     new MaxVelocityConstraint(1.2));

    TrajectoryConfig config = new TrajectoryConfig(
        2.2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics).setReversed(false);

    TrajectoryConfig configBalance = new TrajectoryConfig(
        1.3,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics).setReversed(false);

        // Use this for full motion
    // Trajectory driveToCubeTraj = TrajectoryGenerator.generateTrajectory(
    //     // Start position
    //     startPose,
    //     // Drive to cube
    //     List.of(new Translation2d(2.1, Units.inchesToMeters(108)),
    //         new Translation2d(Units.inchesToMeters(96.75 + 54), Units.inchesToMeters(108)),
    //         new Translation2d(Units.inchesToMeters(96.75 + 54 + 36), Units.inchesToMeters(108))),
    //     // End end at the cube, facing forward
    //     pickupPose,
    //     config);

        Trajectory driveToCubeTraj = TrajectoryGenerator.generateTrajectory(
            // Start position
            startPose,
            // Drive to cube
            List.of(new Translation2d(Units.inchesToMeters(88), Units.inchesToMeters(100)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(108))),
            // End end at the cube, facing forward
            balancePose,
            config);

    // Trajectory driveToBalanceTraj = TrajectoryGenerator.generateTrajectory(
    //     // Start position
    //     pickupPoseFlipped,
    //     // Drive to Center of Charging Station
    //     List.of(new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(108))),
    //     // End end at the cube, facing forward
    //     new Pose2d(Units.inchesToMeters(170), Units.inchesToMeters(108),
    //         new Rotation2d(Math.toRadians(180))),
    //     configBalance);

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

    // SwerveControllerCommand driveToBalance = new SwerveControllerCommand(
    //     driveToBalanceTraj,
    //     drive.poseEstimator::getCurrentPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(2, 0, 0),
    //     new PIDController(2, 0, 0),
    //     thetaController,
    //     drive::setModuleStates,
    //     drive);

    addCommands(
        new InstantCommand(() -> statusLog.append("Starting Blue Mid Cone Park")),

        new InstantCommand(()->{Constants.ConfigConstants.alliance = Alliance.Blue;}), 
        new PlaceFirstCone(drive, arm, gripper, intake, startPose),
        new MoveElbowThenShoulder(arm, Constants.ArmConstants.SAFE_POSITION, 1),


        driveToCube.andThen(() -> drive.drive(0, 0, 0, false)),
        // new InstantCommand(() -> {
        //   drive.turnByAngle(179.99);
        // }, drive),

        //driveToBalance.andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),

        new ChargeStationBalance(drive, 1, 10),
        new InstantCommand(drive::setXModuleState)

    );

  }
}
