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
import frc.robot.customClass.ArmPosition;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class RedRightConeCube extends SequentialCommandGroup {
    /** Creates a new CubeAndLeaveAuto. */
    public RedRightConeCube(DriveSubsystem drive, Arm arm, Gripper gripper, Intake intake) {
        // Field width = 315.5in
        DataLog log = DataLogManager.getLog();
        StringLogEntry statusLog = new StringLogEntry(log, "/my/status");

        // Pose2d startPose = new Pose2d(1.905, Units.inchesToMeters(163.5), new
        // Rotation2d(0));
        Pose2d startPose = new Pose2d(1.905, Units.inchesToMeters(119.5), new Rotation2d(0));
        Pose2d pickupPose = new Pose2d(7.14, Units.inchesToMeters(148.3), new Rotation2d(Math.toRadians(0)));
        Pose2d scoringPose = new Pose2d(2.1, Units.inchesToMeters(141.5), new Rotation2d(0));

        double pickupRange = 0.5;
        Pose2d pickupStart = new Pose2d(pickupPose.getX() - pickupRange, pickupPose.getY(), pickupPose.getRotation());

        RectangularRegionConstraint bumpConstraint = new RectangularRegionConstraint(
                new Translation2d(3.295, Units.inchesToMeters(256)),
                new Translation2d(4.46, Units.inchesToMeters(315.5)),
                new MaxVelocityConstraint(1.0));

        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics).setReversed(false)
                .addConstraint(bumpConstraint)
                .setEndVelocity(1.5);

        TrajectoryConfig configPickup = new TrajectoryConfig(
                AutoConstants.kPickupSpeed,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics).setReversed(false)
                .addConstraint(bumpConstraint);

        TrajectoryConfig configRev = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics).setReversed(true)
                .addConstraint(bumpConstraint);

        Trajectory driveToCubeTraj = TrajectoryGenerator.generateTrajectory(
                // Start position
                startPose,
                // Drive to cube
                List.of(new Translation2d(2.1, Units.inchesToMeters(125.0)),
                        new Translation2d(3.86, Units.inchesToMeters(129.0))),
                // End end at the cube, facing forward
                pickupStart,
                config);

        Trajectory driveToCubeTrajFinish = TrajectoryGenerator.generateTrajectory(
                // Start position
                pickupStart,
                // Drive to cube
                List.of(new Translation2d(pickupStart.getX() + pickupRange / 2, pickupStart.getY())),
                // End end at the cube, facing forward
                pickupPose,
                configPickup);

        driveToCubeTraj = driveToCubeTraj.concatenate(driveToCubeTrajFinish);

        Trajectory driveToPlaceTraj = TrajectoryGenerator.generateTrajectory(
                // Start position
                pickupPose,
                // Drive to cube
                List.of(new Translation2d(3.86, Units.inchesToMeters(129.0)),
                        new Translation2d(2.2, Units.inchesToMeters(129.0))),
                // End end at the cube, facing forward
                scoringPose,
                configRev);

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

        SwerveControllerCommand driveToPlace = new SwerveControllerCommand(
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
                new InstantCommand(() -> statusLog.append("Starting " + this.getName())),

                new InstantCommand(() -> {
                    Constants.ConfigConstants.alliance = Alliance.Red;
                }),
                new PlaceFirstCone(drive, arm, gripper, intake, startPose),
                // new InstantCommand(() -> drive.resetOdometry(startPose)),
                // new InstantCommand(() -> drive.poseEstimator.setCurrentPose(startPose)),

                new InstantCommand(intake::intakeExtend),
                // Drive over line
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new InstantCommand(intake::intakeExtend),
                                new InstantCommand(intake::intakeIn),
                                new InstantCommand(() -> {
                                    intake.runIntake(0.7 * Constants.IntakeConstants.kIntakePowerCone);
                                })),
                        new MoveElbowThenShoulder(arm, ArmConstants.SAFE_POSITION),
                        driveToCube),

                new InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
                new InstantCommand(() -> drive.drive(0, 0, 0, false), drive),

                new WaitCommand(0.5),
                new InstantCommand(intake::intakeOff),
                new InstantCommand(intake::intakeOff),
                new InstantCommand(intake::intakeRetract),

                new ParallelCommandGroup(
                        new GrabAndReadyCube(arm, intake, gripper),
                        driveToPlace),

                new ParallelCommandGroup(
                        new DriveToWall(drive, 0.5),
                        new MultiStepArm(arm, ArmConstants.CUBE_HIGH_POSITION, ArmConstants.CUBE_HIGH_POSITION)),

                new InstantCommand(() -> drive.drive(0, 0, 0, false)),
                new InstantCommand(() -> drive.drive(0, 0, 0, false)),
                new InstantCommand(gripper::openGripper),
                new InstantCommand(gripper::extendKicker),
                new WaitCommand(0.25),
                new InstantCommand(gripper::retractKicker),

                arm.setArmPositionCommand(new ArmPosition(ArmConstants.CUBE_HIGH_POSITION.elbowDegrees - 30,
                        ArmConstants.CUBE_HIGH_POSITION.shoulderDegrees))
        // new InstantCommand(intake::intakeRetract)

        );

    }
}
