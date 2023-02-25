// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
import frc.robot.commands.CubeAndLeaveAutoBlue;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Compressor;
import java.util.List;
import java.util.Map;

import static java.util.Map.entry;

import frc.robot.customClass.CRGB;
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

  private Arm m_Arm = new Arm();
  private boolean elbowInManual = false;
  private boolean shoulderInManual = false;
  private int armInvert = 1;
  private final Gripper m_gripper = new Gripper(true);

  private final Kicker m_Kicker = new Kicker(false);
  
  private Intake m_Intake = new Intake();
  private LED m_Led;


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

  // The controllers - Making two references for backwards compatibility and new Trigger methods
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  CommandXboxController m_manipCommandController = new CommandXboxController(OIConstants.kManipulatorControllerPort);

  POVButton resetGyro = new POVButton(m_driverController, 0); // Up on the D-Pad

  // Drive controllers
  final JoystickButton brakeButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  final TriggerButton lowSpeedTrigger = new TriggerButton(m_driverController, XboxController.Axis.kRightTrigger);

  // Manipulator controllers
  final JoystickButton intakeInButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
  final JoystickButton intakeOutButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);
  final JoystickButton intakeStopButton = new JoystickButton(m_manipulatorController,XboxController.Button.kA.value);
  
  // final POVButton intakePowerCubeButton = new POVButton(m_manipulatorController, 180);
  // final POVButton intakePowerConeButton = new POVButton(m_manipulatorController, 0);
  final POVButton kickerButtonExtend = new POVButton(m_manipulatorController, 0);
  //final POVButton kickerButtonRetract = new POVButton(m_manipulatorController, 180);

  //Arm Position Buttons
  final JoystickButton safePositionButton = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);
  final POVButton pickupPositionButton = new POVButton(m_manipulatorController, 90);
  final POVButton highPositionButton = new POVButton(m_manipulatorController, 270);
  final JoystickButton midPositionButton = new JoystickButton(m_manipulatorController, XboxController.Button.kBack.value);
  final JoystickButton lowPositionButton = new JoystickButton(m_manipulatorController, XboxController.Button.kStart.value);



  private final Trigger gripperButtonClose = m_manipCommandController.rightTrigger();
  private final Trigger gripperButtonOpen = m_manipCommandController.leftTrigger();

  private final JoystickButton intakeExtendButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeRetractButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

  private final SlewRateLimiter elbowPowerLimiter = new SlewRateLimiter(2.0);
  private final SlewRateLimiter shoulderPowerLimiter = new SlewRateLimiter(2.0);


  final JoystickButton testCommandButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);

  PIDController customAnglePID = new PIDController(0.6, 0, 0);
  
  

  private enum CommandsToChoose {
    CubeAndLeaveAutoBlue
  }

  //Create Command variables here for auto
  // public Command shootAndRunLOW;
  // public Command shootAndRunHIGH;
  public Command cubeAndLeaveBlue;

  private final SendableChooser<CommandsToChoose> m_chooser = new SendableChooser<>();
  private Command m_selectCommand = null;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(LED led) {
    SmartDashboard.putString("Robot Type", Constants.robotType.toString());

    m_Led = led;

    //Generate Auto Command Sequences
    generateAutoRoutines();

    //Setup auto command chooser
    m_selectCommand = new SelectCommand(Map.ofEntries(
        entry(CommandsToChoose.CubeAndLeaveAutoBlue, cubeAndLeaveBlue)
       ), m_chooser::getSelected);

    m_chooser.setDefaultOption("Shoot and Run Low", CommandsToChoose.CubeAndLeaveAutoBlue);
    
    Shuffleboard.getTab("Auto").add(m_chooser)
    .withPosition(0, 0)
    .withSize(7, 2);

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
   
    intakeInButton.onTrue(new InstantCommand(m_Intake::intakeIn, m_Intake));
    intakeStopButton.onTrue(new InstantCommand(m_Intake::intakeOff, m_Intake));
    intakeOutButton.onTrue(new InstantCommand(m_Intake::intakeOut, m_Intake));

    // intakePowerCubeButton.onTrue(new InstantCommand(m_Intake::setIntakePowerCube, m_Intake)
    //     .andThen(new InstantCommand(m_Led::setPurple, m_Led)));
    // intakePowerConeButton.onTrue(new InstantCommand(m_Intake::setIntakePowerCone, m_Intake)
    //     .andThen(new InstantCommand(m_Led::setYellow, m_Led)));
    
    intakeExtendButton.onTrue(new InstantCommand(m_Intake::intakeExtend, m_Intake));
    intakeRetractButton.onTrue(new InstantCommand(m_Intake::intakeRetract, m_Intake));

    gripperButtonOpen.onTrue(new InstantCommand(m_gripper::openGripper, m_gripper));
    gripperButtonClose.onTrue(new InstantCommand(m_gripper::closeGripper, m_gripper)
        .andThen(new InstantCommand(m_Intake::magicCarpetOff, m_Intake)));

    kickerButtonExtend.onTrue(new InstantCommand(m_Kicker::extendKicker, m_Kicker))
        .onFalse(new InstantCommand(m_Kicker::retractKicker, m_Kicker));
    //kickerButtonRetract.onTrue(new InstantCommand(m_Kicker::retractKicker, m_Kicker));

    safePositionButton.onTrue(m_Arm.setArmPositionCommand(Constants.ArmConstants.SAFE_POSITION));
    pickupPositionButton.onTrue(m_Arm.setArmPositionCommand(Constants.ArmConstants.PICKUP_POSITION));
    highPositionButton.onTrue(m_Arm.setArmPositionCommand(Constants.ArmConstants.HIGH_POSITION));
    midPositionButton.onTrue(m_Arm.setArmPositionCommand(Constants.ArmConstants.MID_POSITION));
    lowPositionButton.onTrue(m_Arm.setArmPositionCommand(Constants.ArmConstants.LOW_POSITION));

    Runnable Control = () -> {
      if (m_robotDrive != null) {
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
        // m_robotDrive.zeroHeading();
        // System.out.println("Zeroing");
        // }

        // Call the Method
        // SmartDashboard.putNumber("xSpeed", xSpeed);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
        // SmartDashboard.putNumber("rotation", rotation);
        SmartDashboard.putBoolean("Field Rel", fieldRelative);
        m_robotDrive.drive(xSpeed, ySpeed, rotation, fieldRelative);

        // System.out.println("Starting Pose Angle" +
        // m_robotDrive.getPose().getRotation().getDegrees());

      }
    };



    m_robotDrive.setDefaultCommand(new RunCommand(Control, m_robotDrive));
    m_Arm.setDefaultCommand(new RunCommand(ControlArm, m_Arm));
    

    lowSpeedTrigger.onTrue(new InstantCommand(m_robotDrive::setSlowDrive, m_robotDrive))
         .onFalse(new InstantCommand(m_robotDrive::setNormalDrive, m_robotDrive));
    
    resetGyro.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    brakeButton.whileTrue(new RunCommand(m_robotDrive::setXModuleState, m_robotDrive));

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
    cubeAndLeaveBlue = new CubeAndLeaveAutoBlue(m_robotDrive, m_Arm, m_gripper, m_Intake);
    // shootAndRunHIGH = new ShootandRunHIGH(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    // grabShootShoot = new GrabShootShoot(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
    // wallGrabShootShoot = new WallGrabShootShoot(m_robotDrive, m_shooter, m_intakePackage, m_intake, m_intestine, m_climber);
  }

  public void zeroOdo(){
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  Runnable ControlArm = () -> {
    // Arm Control
    // if(!shoulderInManual){
    //   armInvert = (m_Arm.getShoulderPositionDeg() < 200.0) ? -1 : 1;
    // } 

    double shoulderPower = armInvert * shoulderPowerLimiter.calculate(new_deadzone(m_manipulatorController.getLeftY()));
    double elbowPower = armInvert * elbowPowerLimiter.calculate(new_deadzone(m_manipulatorController.getRightY()));

    //The following violates the intent of Command-based and should
    //be modified to use Commands
    if(shoulderPower != 0){
      shoulderInManual = true;
      m_Arm.runShoulder(shoulderPower);
    } else if(shoulderInManual) {
      shoulderInManual = false;
      m_Arm.runShoulder(0); // Remove once hold function is stable
    }
    
    if (elbowPower != 0) {
      elbowInManual = true;
      m_Arm.runElbow(elbowPower);
    } else if (elbowInManual) {
      elbowInManual = false;
      m_Arm.runElbow(0); // Remove once hold function is stable
    }

    // m_manipCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.25)
    //   .and(m_manipCommandController.axisLessThan(XboxController.Axis.kRightTrigger.value, 0.5))
    //   .onTrue(new InstantCommand(m_Arm::holdElbowPosition, m_Arm));
    
  };
  
}