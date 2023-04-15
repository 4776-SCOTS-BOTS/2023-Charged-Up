package frc.robot.commands;

import java.util.function.Supplier;

import javax.swing.plaf.nimbus.State;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightCubeTracker;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class ChaseCube extends CommandBase {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 3);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 3);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final double YSCALE_FACTOR = 70; // units from cube chaser per foot - rough

    private final DriveSubsystem drive;
    private LimeLightCubeTracker cubeTracker;
    private double xSpeedStart;
    private Pose2d targetPose, updatedTargetPose;
    private final Supplier<Pose2d> poseProvider;

    private Timer timer = new Timer();
    private double startTime, timeout;
    private boolean atGoal = false;
    
    DataLog log = DataLogManager.getLog();
    StringLogEntry statusLog = new StringLogEntry(log, "/my/status");
    StringLogEntry chaseLog = new StringLogEntry(log, "/my/chaseLog");

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    // private PhotonTrackedTarget lastTarget;

    // Assume we ended last move with only a residual forward motion
    public ChaseCube(
            DriveSubsystem drive, Pose2d targetPose, double xSpeed, double timeout) {

        this.drive = drive;
        this.poseProvider = drive.poseEstimator::getCurrentPose;
        this.targetPose = targetPose;
        this.xSpeedStart = xSpeed;
        this.timeout = timeout;
        this.cubeTracker = new LimeLightCubeTracker("limelight-cubecam");

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        statusLog.append("Starting Cube Chase");
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX(), xSpeedStart);
        yController.reset(robotPose.getY());
        updatedTargetPose = targetPose;
        startTime = timer.getFPGATimestamp();
        atGoal = false;
    }

    @Override
    public void execute() {
        atGoal = true;
        var robotPose2d = poseProvider.get();

        double online = cubeTracker.cubeFromCenterLine();

        if (Math.abs(online) < 200) {
            double yoffset = Units.feetToMeters(online / YSCALE_FACTOR);
            updatedTargetPose = new Pose2d(targetPose.getX(), robotPose2d.getY() + yoffset, targetPose.getRotation());
        }

        // Calculate Drive
        xController.setGoal(updatedTargetPose.getX());
        yController.setGoal(updatedTargetPose.getY());
        omegaController.setGoal(updatedTargetPose.getRotation().getRadians());

        var xSpeed = phaseStartSpeed(xController.calculate(robotPose2d.getX()));
        if (xController.atGoal()) {
            xSpeed = 0;
        } else {
            atGoal = false;
        }

        var ySpeed = yController.calculate(robotPose2d.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        } else {
            atGoal = false;
        }

        var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        } else {
            atGoal = false;
        }

        chaseLog.append("xSpeed: " + xSpeed + " / yspeed: " + ySpeed + " / rotSpeed: " + omegaSpeed);
        drive.drive(xSpeed, ySpeed, omegaSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished(){
        return (Timer.getFPGATimestamp() - startTime > timeout) || atGoal;
    }

    public double phaseStartSpeed(double trapSpeed){
        double phaseTime = 0.5;
        double phaseMinSpeed = 1.0;
        double slope = ((xSpeedStart - phaseMinSpeed) / phaseTime);
        double elapsedTime = timer.getFPGATimestamp() - startTime;

        if(elapsedTime > phaseTime){
            return trapSpeed;
        } else {
            double tempSpeed = xSpeedStart - elapsedTime*slope;
            if (tempSpeed > trapSpeed){
                return tempSpeed;
            } else {
                return trapSpeed;
            }
        }

    }

}