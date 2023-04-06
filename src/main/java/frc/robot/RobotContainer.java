// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BlueLeftCone;
import frc.robot.commands.BlueLeftConeCube;
import frc.robot.commands.BlueMidConePark;
import frc.robot.commands.BlueRightCone;
import frc.robot.commands.BlueRightConeCube;
import frc.robot.commands.ChargeStationBalance;
import frc.robot.commands.CurrentSenseIntake;
import frc.robot.commands.MoveElbowThenShoulder;
import frc.robot.commands.MoveShoulderThenElbow;
import frc.robot.commands.MultiStepArm;
import frc.robot.commands.RedLeftCone;
import frc.robot.commands.RedLeftConeCube;
import frc.robot.commands.RedMidConePark;
import frc.robot.commands.RedRightCone;
import frc.robot.commands.RedRightConeCube;
import frc.robot.customClass.TriggerButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LED;

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
  private boolean conePositions = true;

  private Arm m_Arm = new Arm();
  private boolean elbowInManual = false;
  private boolean shoulderInManual = false;
  private int armInvert = 1;
  private final Kicker m_Kicker = new Kicker(false);
  private final Gripper m_gripper = new Gripper(true, m_Kicker);

  

  private Intake m_Intake = new Intake();
  private LED m_Led;

  // private PoseEstimatorSubsystem poseEstimator = new
  // PoseEstimatorSubsystem("limelight", m_robotDrive);

  // The controllers - Making two references for backwards compatibility and new
  // Trigger methods
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  CommandXboxController m_manipCommandController = new CommandXboxController(OIConstants.kManipulatorControllerPort);

  POVButton resetGyro = new POVButton(m_driverController, 0); // Up on the D-Pad

  // Drive controllers
  final JoystickButton brakeButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  final TriggerButton lowSpeedTrigger = new TriggerButton(m_driverController, XboxController.Axis.kRightTrigger);
  final TriggerButton reallylowSpeedTrigger = new TriggerButton(m_driverController, XboxController.Axis.kLeftTrigger);
  final JoystickButton signalCubeButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  final JoystickButton signalConeButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
 
  // Manipulator controllers
  final JoystickButton intakeInButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
  final JoystickButton intakeOutButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);
  final JoystickButton intakeStopButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
  
  

  // Arm Position Buttons
  final JoystickButton safePositionButton = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);
  final POVButton kickerButtonExtend = new POVButton(m_manipulatorController, 0);
  final POVButton currentSenseIntakeButton = new POVButton(m_manipulatorController, 90);
  final POVButton armStopButton = new POVButton(m_manipulatorController, 180);
  final POVButton highPositionButton = new POVButton(m_manipulatorController, 270);

  final JoystickButton midPositionButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kBack.value);
  final JoystickButton pickupPositionConeButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kLeftStick.value);
  final JoystickButton pickupPositionCubeButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kRightStick.value);
  final JoystickButton tipperButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kStart.value);

  private final Trigger gripperButtonClose = m_manipCommandController.rightTrigger();
  private final Trigger gripperButtonOpen = m_manipCommandController.leftTrigger();

  private final JoystickButton intakeExtendButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeRetractButton = new JoystickButton(m_manipulatorController,
      XboxController.Button.kRightBumper.value);

  private static Trigger triggerStick(XboxController controller, XboxController.Axis axis) {
    return new Trigger(() -> Math.abs(controller.getRawAxis(axis.value)) >= 0.1);
  }
  private static Trigger triggerStickPos(XboxController controller, XboxController.Axis axis) {
    return new Trigger(() -> controller.getRawAxis(axis.value) >= 0.7);
  }
  private static Trigger triggerStickNeg(XboxController controller, XboxController.Axis axis) {
    return new Trigger(() -> controller.getRawAxis(axis.value) <= -0.7);
  }

  private final Trigger elbowStick = triggerStick(m_manipulatorController, XboxController.Axis.kRightY);
  private final Trigger shoulderStick = triggerStick(m_manipulatorController, XboxController.Axis.kLeftY);
  private final Trigger cubeLowStick = triggerStickNeg(m_manipulatorController, XboxController.Axis.kRightX);
  private final Trigger cubeHighStick = triggerStickPos(m_manipulatorController, XboxController.Axis.kRightX);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

  private final SlewRateLimiter elbowPowerLimiter = new SlewRateLimiter(2.0);
  private final SlewRateLimiter shoulderPowerLimiter = new SlewRateLimiter(2.0);

  // Test Buttons
  // AUTO LEVEL BUTTON BINDING
  final JoystickButton testCommandButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  // final JoystickButton testPoseSetButton = new
  // JoystickButton(m_driverController, XboxController.Button.kBack.value);

  PIDController customAnglePID = new PIDController(0.6, 0, 0);

  private enum CommandsToChoose {
    BlueRightCone,
    BlueMidConePark,
    BlueLeftCone,
    RedRightCone,
    RedMidConePark,
    RedLeftCone,
    BlueRightConeCube,
    BlueLeftConeCube,
    RedRightConeCube,
    RedLeftConeCube
    // BlueRightCube,
    // BlueMidCubePark,
    // BlueLeftCube,
    // RedRightCube,
    // RedMidCubePark,
    // RedLeftCube,
  }

  private enum AllianceToChoose {
    Blue,
    Red
  }

  private enum ElementToPlace {
    Cone,
    Cube
  }

  private enum GridToPlace {
    Left,
    Mid,
    Right
  }

  // Create Command variables here for auto
  public Command blueRightCone;
  public Command blueMidConePark;
  public Command blueLeftCone;
  public Command redRightCone;
  public Command redMidConePark;
  public Command redLeftCone;
  public Command blueRightConeCube;
  public Command blueLeftConeCube;
  public Command redRightConeCube;
  public Command redLeftConeCube;
  // public Command blueRightCube;
  // public Command blueMidCubePark;
  // public Command blueLeftCube;
  // public Command redRightCube;
  // public Command redMidCubePark;
  // public Command redLeftCube;

  private final SendableChooser<CommandsToChoose> m_chooser = new SendableChooser<>();
  private Command m_selectCommand = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(LED led) {
    SmartDashboard.putString("Robot Type", Constants.robotType.toString());
    // SmartDashboard.putNumber("Remaining Time", DriverStation.getMatchTime());

    m_Led = led;

    // Generate Auto Command Sequences
    generateAutoRoutines();

    // Setup auto command chooser
    m_selectCommand = new SelectCommand(Map.ofEntries(
        entry(CommandsToChoose.BlueRightCone, blueRightCone),
        entry(CommandsToChoose.BlueMidConePark, blueMidConePark),
        entry(CommandsToChoose.BlueLeftCone, blueLeftCone),
        entry(CommandsToChoose.RedRightCone, redRightCone),
        entry(CommandsToChoose.RedMidConePark, redMidConePark),
        entry(CommandsToChoose.RedLeftCone, redLeftCone),
        // entry(CommandsToChoose.BlueRightCube, blueRightCube),
        // entry(CommandsToChoose.BlueMidCubePark, blueMidCubePark),
        // entry(CommandsToChoose.BlueLeftCube, blueLeftCube),
        // entry(CommandsToChoose.RedRightCube, redRightCube),
        // entry(CommandsToChoose.RedMidCubePark, redMidCubePark),
        // entry(CommandsToChoose.RedLeftCube, redLeftCube),
        entry(CommandsToChoose.BlueRightConeCube, blueRightConeCube),
        entry(CommandsToChoose.BlueLeftConeCube, blueLeftConeCube),
        entry(CommandsToChoose.RedRightConeCube, redRightConeCube),
        entry(CommandsToChoose.RedLeftConeCube, redLeftConeCube)), m_chooser::getSelected);

    m_chooser.setDefaultOption("Blue: Right Cone+Cube", CommandsToChoose.BlueRightConeCube);
    m_chooser.addOption("Blue: Mid Cone and Balance", CommandsToChoose.BlueMidConePark);
    m_chooser.addOption("Blue: Left Cone+Cube", CommandsToChoose.BlueLeftConeCube);

    m_chooser.addOption("Blue: Right Cone", CommandsToChoose.BlueRightCone);    
    m_chooser.addOption("Blue: Left Cone", CommandsToChoose.BlueLeftCone);

    m_chooser.addOption("Red: Right Cone+Cube", CommandsToChoose.RedRightConeCube);
    m_chooser.addOption("Red: Mid Cone and Balance", CommandsToChoose.RedMidConePark);
    m_chooser.addOption("Red: Left Cone+Cube", CommandsToChoose.RedLeftConeCube);

    m_chooser.addOption("Red: Right Cone", CommandsToChoose.RedRightCone);
    m_chooser.addOption("Red: Left Cone", CommandsToChoose.RedLeftCone);

    // m_chooser.addOption("Blue: Right Cube", CommandsToChoose.BlueRightCube);
    // m_chooser.addOption("Blue: Mid Cube and Balance", CommandsToChoose.BlueMidCubePark);
    // m_chooser.addOption("Blue: Left Cube", CommandsToChoose.BlueLeftCube);

    // m_chooser.addOption("Red: Right Cube", CommandsToChoose.RedRightCube);
    // m_chooser.addOption("Red: Mid Cube and Balance", CommandsToChoose.RedMidCubePark);
    // m_chooser.addOption("Red: Left Cube", CommandsToChoose.RedLeftCube);

    Shuffleboard.getTab("Auto").add(m_chooser)
        .withPosition(0, 2)
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

    signalConeButton.onTrue(new InstantCommand(m_Led::setYellow, m_Led));
    signalCubeButton.onTrue(new InstantCommand(m_Led::setPurple, m_Led)); 

    intakeInButton.onTrue(new InstantCommand(m_Intake::intakeIn, m_Intake));
    intakeStopButton.onTrue(new InstantCommand(m_Intake::intakeOff, m_Intake));
    intakeOutButton.onTrue(new InstantCommand(m_Intake::intakeOut, m_Intake));

    tipperButton.onTrue(
        m_Arm.setArmPositionCommand(ArmConstants.AUTO_TIPPER)
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(m_Intake::magicCarpetOut, m_Intake))
            .andThen(new WaitCommand(2))
            .andThen(new InstantCommand(m_Intake::magicCarpetIn, m_Intake))
            .andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(m_Intake::magicCarpetOff, m_Intake))
            .andThen(new InstantCommand(m_Intake::magicCarpetOff, m_Intake)));
   // tipperButton.whileFalse(new InstantCommand(m_Intake::tipperSafe, m_Intake));

    // intakePowerCubeButton.onTrue(new InstantCommand(m_Intake::setIntakePowerCube,
    // m_Intake)
    // .andThen(new InstantCommand(m_Led::setPurple, m_Led)));
    // intakePowerConeButton.onTrue(new InstantCommand(m_Intake::setIntakePowerCone,
    // m_Intake)
    // .andThen(new InstantCommand(m_Led::setYellow, m_Led)));

    intakeExtendButton.onTrue(new InstantCommand(m_Intake::intakeExtend, m_Intake));
    intakeRetractButton.onTrue(new InstantCommand(m_Intake::intakeRetract, m_Intake));

    gripperButtonOpen.onTrue(new InstantCommand(m_gripper::openGripper, m_gripper));
    gripperButtonClose.onTrue(new InstantCommand(m_gripper::closeGripper, m_gripper)
        .andThen(new InstantCommand(m_Intake::magicCarpetOff, m_Intake)));

    kickerButtonExtend.onTrue(new InstantCommand(m_Kicker::extendKicker, m_Kicker))
        .onFalse(new InstantCommand(m_Kicker::retractKicker, m_Kicker));
    // kickerButtonRetract.onTrue(new InstantCommand(m_Kicker::retractKicker,
    // m_Kicker));

    safePositionButton.onTrue(new MoveElbowThenShoulder(m_Arm, Constants.ArmConstants.SAFE_POSITION)
      .andThen(new InstantCommand(m_gripper::closeGripper, m_gripper)));
    
    armStopButton.onTrue(new InstantCommand( ()->{
      shoulderInManual = true;
      m_Arm.runShoulder(0);
      elbowInManual = true;
        m_Arm.runElbow(0);
    }, m_Arm));

    currentSenseIntakeButton.onTrue(new CurrentSenseIntake(m_Intake)); 

        highPositionButton.onTrue(new SelectCommand(Map.ofEntries(
          Map.entry(true, //m_Arm.setArmPositionCommand(ArmConstants.HIGH_POSITION)),
          new MoveShoulderThenElbow(m_Arm, Constants.ArmConstants.HIGH_POSITION)),
          Map.entry(false, new MoveShoulderThenElbow(m_Arm, Constants.ArmConstants.CUBE_HIGH_POSITION))),
          () -> conePositions));
        
    midPositionButton.onTrue(new SelectCommand(Map.ofEntries(
        Map.entry(true, m_Arm.setArmPositionCommand(Constants.ArmConstants.MID_POSITION)),
        Map.entry(false, m_Arm.setArmPositionCommand(Constants.ArmConstants.CUBE_MID_POSITION))),
        () -> conePositions));

    pickupPositionConeButton.onTrue(new InstantCommand(m_gripper::openGripper)
        .andThen(new MultiStepArm(m_Arm, Constants.ArmConstants.PICKUP_POSITION1,
            Constants.ArmConstants.PICKUP_POSITION))
        .andThen(new InstantCommand(() -> {
          conePositions = true;
        })));
    pickupPositionCubeButton.onTrue(new InstantCommand(m_gripper::openGripper)
        .andThen(new MoveElbowThenShoulder(m_Arm, ArmConstants.PICKUP_POSITION_CUBE, 1))
        .andThen(new InstantCommand(() -> {
          conePositions = false;
        })));

    elbowStick.onTrue(new InstantCommand(() -> {
      elbowInManual = true;
      shoulderInManual = true;
    }, m_Arm).andThen(
        new InstantCommand(ControlArm, m_Arm)));
    shoulderStick.onTrue(new InstantCommand(() -> {
      elbowInManual = true;
      shoulderInManual = true;
    }, m_Arm).andThen(
        new InstantCommand(ControlArm, m_Arm)));

    cubeLowStick.onTrue(new InstantCommand(()->{m_Intake.runIntake(Constants.IntakeConstants.kSpitCubeLow);}));
    cubeHighStick.onTrue(new InstantCommand(()->{m_Intake.runIntake(Constants.IntakeConstants.kSpitCubeHigh);}));

    // standingConePickupButton.onTrue(new StandingCone(m_Arm, m_gripper, m_Intake)
    //     .andThen(m_Arm.setArmPositionCommand(Constants.ArmConstants.PICKUP_STANDING_CONE)));

    Runnable Control = () -> {
      if (m_robotDrive != null) {
        SmartDashboard.putNumber("Remaining Time", DriverStation.getMatchTime());
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

    reallylowSpeedTrigger.onTrue(new InstantCommand(m_robotDrive::setReallySlowDrive, m_robotDrive))
        .onFalse(new InstantCommand(m_robotDrive::setNormalDrive, m_robotDrive));

    resetGyro.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    brakeButton.whileTrue(new RunCommand(m_robotDrive::setXModuleState, m_robotDrive));
    // AUTO BALANCE COMMAND
    testCommandButton.onTrue(new ChargeStationBalance(m_robotDrive, 3, 10));

    // m_robotDrive.turnByAngle(179.9);
    // }, m_robotDrive));

    // testCommandButton.whileHeld(new InstantCommand(()->{
    // m_robotDrive.coastModuleTurn();
    // }, m_robotDrive));

    //testPoseSetButton.onTrue(new InstantCommand(()-> {m_robotDrive.poseEstimator.setAllPoseToLimeLight();}));

  }

  double new_deadzone(double x) {
    if (Math.abs(x) > 0.06) {
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
    blueRightConeCube = new BlueRightConeCube(m_robotDrive, m_Arm, m_gripper, m_Intake);
    blueLeftConeCube = new BlueLeftConeCube(m_robotDrive, m_Arm, m_gripper, m_Intake);

    redRightConeCube = new RedRightConeCube(m_robotDrive, m_Arm, m_gripper, m_Intake);
    redLeftConeCube = new RedLeftConeCube(m_robotDrive, m_Arm, m_gripper, m_Intake);
    
    blueRightCone = new BlueRightCone(m_robotDrive, m_Arm, m_gripper, m_Intake);
    blueMidConePark = new BlueMidConePark(m_robotDrive, m_Arm, m_gripper, m_Intake);
    blueLeftCone = new BlueLeftCone(m_robotDrive, m_Arm, m_gripper, m_Intake);
    redRightCone = new RedRightCone(m_robotDrive, m_Arm, m_gripper, m_Intake);
    redMidConePark = new RedMidConePark(m_robotDrive, m_Arm, m_gripper, m_Intake);
    redLeftCone = new RedLeftCone(m_robotDrive, m_Arm, m_gripper, m_Intake);
    // blueRightCube = new BlueRightCube(m_robotDrive, m_Arm, m_gripper, m_Intake);
    // blueMidCubePark = new BlueMidCubePark(m_robotDrive, m_Arm, m_gripper, m_Intake);
    // blueLeftCube = new BlueLeftCube(m_robotDrive, m_Arm, m_gripper, m_Intake);
    // redRightCube = new RedRightCube(m_robotDrive, m_Arm, m_gripper, m_Intake);
    // redMidCubePark = new RedMidCubePark(m_robotDrive, m_Arm, m_gripper, m_Intake);
    // redLeftCube = new RedLeftCube(m_robotDrive, m_Arm, m_gripper, m_Intake);
  }

  public void zeroOdo() {
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  Runnable ControlArm = () -> {
    // Arm Control
    // if(!shoulderInManual){
    // armInvert = (m_Arm.getShoulderPositionDeg() < 200.0) ? -1 : 1;
    // }

    double shoulderPower = Math.pow(armInvert * new_deadzone(m_manipulatorController.getLeftY()),5);
    double elbowPower = Math.pow(armInvert * new_deadzone(m_manipulatorController.getRightY()),5);

    // The following violates the intent of Command-based and should
    // be modified to use Commands
    if (shoulderPower != 0) {
      shoulderInManual = true;
      m_Arm.runShoulder(shoulderPower);
    } else if (shoulderInManual) {
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

    // m_manipCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value,
    // 0.25)
    // .and(m_manipCommandController.axisLessThan(XboxController.Axis.kRightTrigger.value,
    // 0.5))
    // .onTrue(new InstantCommand(m_Arm::holdElbowPosition, m_Arm));


  };

}