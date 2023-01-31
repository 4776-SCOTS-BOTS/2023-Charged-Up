// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;
import java.util.Map;

import static java.util.Map.entry;

import frc.robot.customClass.TriggerButton;
import frc.robot.subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private boolean fieldRelative = true;

  private Arm arm = new Arm();
  private boolean elbowInManual = false;
  private boolean shoulderInManual = false;

  // Init Limelight
  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry txShuffle = table.getEntry("tx");
  // NetworkTableEntry tyShuffle = table.getEntry("ty");
  // NetworkTableEntry taShuffle = table.getEntry("ta");

  // Read the Limelight values periodically
  // double x = txShuffle.getDouble(0.0);
  // double y = tyShuffle.getDouble(0.0);
  // double area = taShuffle.getDouble(0.0);

  /** Creates a new LimelightSubsystem. */
  // public void LimelightSubsystem() {
  // setupLimelightShuffleBoard();
  // }

  // The controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  POVButton resetGyro = new POVButton(m_driverController, 0);

  final JoystickButton brakeButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  final TriggerButton lowSpeedTrigger = new TriggerButton(m_driverController, XboxController.Axis.kRightTrigger);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

  private final SlewRateLimiter elbowPowerLimiter = new SlewRateLimiter(1, 3, 0);
  private final SlewRateLimiter shoulderPowerLimiter = new SlewRateLimiter(1, 3, 0);

  final JoystickButton testCommandButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);

  PIDController customAnglePID = new PIDController(0.6, 0, 0);

  private enum CommandsToChoose {
    ShootandRunLOW, ShootandRunHIGH, GrabShootShoot, WallGrabShootShoot
  }

  //Create Command variables here for auto
  // public Command shootAndRunLOW;
  // public Command shootAndRunHIGH;

  private final SendableChooser<CommandsToChoose> m_chooser = new SendableChooser<>();
  private Command m_selectCommand = null;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putString("Robot Type", Constants.robotType.toString());

    // Generate Auto Command Sequences
    //generateAutoRoutines();

    // Setup auto command chooser
    // m_selectCommand = new SelectCommand(Map.ofEntries(
    //     entry(CommandsToChoose.ShootandRunLOW, shootAndRunLOW),
    //     entry(CommandsToChoose.ShootandRunHIGH, shootAndRunHIGH),
    //     entry(CommandsToChoose.GrabShootShoot, grabShootShoot),
    //     entry(CommandsToChoose.WallGrabShootShoot, wallGrabShootShoot)), m_chooser::getSelected);

    // m_chooser.setDefaultOption("Shoot and Run Low", CommandsToChoose.ShootandRunLOW);
    // m_chooser.addOption("Shoot and Run High", CommandsToChoose.ShootandRunHIGH);
    // m_chooser.addOption("Grab Ball and Shoot Two", CommandsToChoose.GrabShootShoot);
    // m_chooser.addOption("Wall Grab Ball and Shoot Two", CommandsToChoose.WallGrabShootShoot);

    // Shuffleboard.getTab("Auto").add(m_chooser)
    // .withPosition(0, 0)
    // .withSize(7, 2);

    // Configure the button bindings
    configureButtonBindings();

  }

  // private void setupLimelightShuffleBoard() {
  // final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  // txShuffle = limelightTab.add("LimelightX", x).getEntry();
  // tyShuffle = limelightTab.add("LimelightY", y).getEntry();
  // taShuffle = limelightTab.add("LimelightArea", area).getEntry();
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    customAnglePID.enableContinuousInput(-Math.PI, Math.PI);

    Runnable Control = () -> {

      // SmartDashboard.putNumber("LeftY", m_driverController.getLeftY());
      // SmartDashboard.putNumber("LeftX", m_driverController.getLeftX());
      // SmartDashboard.putNumber("RightX", m_driverController.getRightX());

      // Swerve xSpeed is the vertical/forward (negative because stick is inverse)
      double xSpeed = DriveConstants.drivePercentScale * DriveConstants.kMaxSpeedMetersPerSecond
          * xSpeedLimiter.calculate(new_deadzone(-m_driverController.getLeftY()));

      // Swerve ySpeed is the sideways left/right movement (negative because chassis
      // +y is LEFT)
      double ySpeed = DriveConstants.drivePercentScale * DriveConstants.kMaxSpeedMetersPerSecond
          * ySpeedLimiter.calculate(new_deadzone(-m_driverController.getLeftX()));

      // Swerve rotation is the counter-clockwise rotation of the robot (negate stick
      // input)
      double rotation = DriveConstants.drivePercentScale * DriveConstants.kMaxSpeedMetersPerSecond
          * DriveConstants.rotRateModifier * rotLimiter.calculate(new_deadzone(-m_driverController.getRightX()));

      // Swap field/robot relative mode
      if (m_driverController.getLeftBumper()) {
        fieldRelative = true;
      } else if (m_driverController.getRightBumper()) {
        fieldRelative = false;
      }

      // Reset Gyro
      // if (m_driverController.getPOV() == 0) {
      //   m_robotDrive.zeroHeading();
      //   System.out.println("Zeroing");
      // }

      // Call the Method
      // SmartDashboard.putNumber("xSpeed", xSpeed);
      // SmartDashboard.putNumber("ySpeed", ySpeed);
      // SmartDashboard.putNumber("rotation", rotation);
      SmartDashboard.putBoolean("Field Rel", fieldRelative);
      m_robotDrive.drive(xSpeed, ySpeed, rotation, fieldRelative);

      // System.out.println("Starting Pose Angle" +
      // m_robotDrive.getPose().getRotation().getDegrees());

    };
    m_robotDrive.setDefaultCommand(new RunCommand(Control, m_robotDrive));

    lowSpeedTrigger.onTrue(new InstantCommand(m_robotDrive::setSlowDrive, m_robotDrive))
        .onFalse(new InstantCommand(m_robotDrive::setNormalDrive, m_robotDrive));
    
    resetGyro.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    brakeButton.whileTrue(new InstantCommand(m_robotDrive::setXModuleState, m_robotDrive));

    // testCommandButton.whenPressed(new InstantCommand(()->{
    // m_robotDrive.turnByAngle(179.9);
    // }, m_robotDrive));

    // testCommandButton.whileHeld(new InstantCommand(()->{
    //   m_robotDrive.coastModuleTurn();
    //   }, m_robotDrive));

  }

  double new_deadzone(double x) {
    if (Math.abs(x) > 0.08) {
      return x;
    } else {
      return 0;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_selectCommand;
  }

 
  // Generate auto routines
  public void generateAutoRoutines() {
    // shootAndRunLOW = new ShootandRunLOW(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    // shootAndRunHIGH = new ShootandRunHIGH(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    // grabShootShoot = new GrabShootShoot(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    // wallGrabShootShoot = new WallGrabShootShoot(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
  }

  public void zeroOdo(){
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  Runnable ControlArm = () -> {
    // Arm Control
    double shoulderPower = shoulderPowerLimiter.calculate(new_deadzone(-m_manipulatorController.getLeftY()));
    double elbowPower = elbowPowerLimiter.calculate(new_deadzone(-m_manipulatorController.getLeftY()));

    //This following violates the intent of Command-based and should
    //modified to use Commands
    if(shoulderPower != 0){
      shoulderInManual = true;
      arm.runShoulder(shoulderPower);
    } else if(shoulderInManual) {
      //Do nothing other than stop the shoulder as shoulder control code is not done
      arm.runShoulder(0); //Remove this line once control code is done
    }

    if(elbowPower!=0){
      elbowInManual = true;
      arm.runElbow(elbowPower);
    } else if(elbowInManual) {
      elbowInManual = false;  
      arm.holdElbowPosition();
    } else if(m_manipulatorController.){
      double a =1;
    }
    
    
  };
  
}