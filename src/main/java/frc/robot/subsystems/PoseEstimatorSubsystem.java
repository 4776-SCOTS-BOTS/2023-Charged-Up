package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.customClass.TimestampedBotPose3d;
import frc.robot.subsystems.LimeLightPoseEstimator;
import frc.robot.subsystems.DriveSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final LimeLightPoseEstimator limelight;
  private final DriveSubsystem drive;
  private TimestampedBotPose3d currentLimeLightPose;
  // private final AprilTagFieldLayout aprilTagFieldLayout;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   * This is for the robot drivetrain odometry.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10));
  private static final Vector<N3> visionMeasurementStdDevsClose = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(10));
  private static final Vector<N3> visionMeasurementStdDevsMid = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10));
  private static final Vector<N3> visionMeasurementStdDevsFar = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

  //Range Definitions if less than these values (meters)
  private static final double RANGE_CLOSE = 1.5;
  private static final double RANGE_MEDIUM = 4;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;
  private Pose2d previousPose = new Pose2d();
  private Translation2d previousTrans = new Translation2d();
  private double distancePrev = 0;
  private double tagDistance = 0;

  public PoseEstimatorSubsystem(String limelightName, DriveSubsystem drive) {
    this.limelight = new LimeLightPoseEstimator(limelightName);
    this.drive = drive;
    // AprilTagFieldLayout layout;
    // try {
    //   layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    //   var alliance = DriverStation.getAlliance();
    //   layout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
    //       : OriginPosition.kRedAllianceWallRightSide);
    // } catch (IOException e) {
    //   DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    //   layout = null;
    // }
    // this.aprilTagFieldLayout = layout;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    Pose2d startPose = new Pose2d();
    previousPose = startPose;
    previousTrans = startPose.getTranslation();

    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        drive.getGyroRotation(),
        drive.getModulePositions(),
        startPose,
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose", this::getFormattedPose).withPosition(0, 0).withSize(4, 2);
    tab.addString("LimePose", this::getFormattedLimePose).withPosition(0, 3).withSize(4, 2);
    tab.addDouble("Tag Distance", this::getTagDistance).withPosition(0, 6).withSize(4, 2);
    tab.add("Field", field2d).withPosition(5, 0).withSize(10, 8);
  }

  @Override
  public void periodic() {
    double distanceLimit = 0.5;
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        drive.getGyroRotation(),
        drive.getModulePositions());

    TimestampedBotPose3d[] timestampedPoses3d = limelight.getBotPose3d();

    // for (TimestampedBotPose3d pose : timestampedPoses3d) {
    currentLimeLightPose = timestampedPoses3d[0];
    distancePrev = previousTrans.getDistance(currentLimeLightPose.pose3d.toPose2d().getTranslation());
    tagDistance = currentLimeLightPose.tagDistance;
    //System.out.println("Target Distance = " + tagDistance);
    
    if(tagDistance <= RANGE_CLOSE){
      visionMeasurementStdDevs = visionMeasurementStdDevsClose;
      distanceLimit = 10; //Trust the limelight pose even if odometry disagrees
    } else if(tagDistance <= RANGE_MEDIUM){
      visionMeasurementStdDevs = visionMeasurementStdDevsMid;
      distanceLimit = 0.1; //Gets jumpy in this range and need to filter.  Only using good agreement
    } else {
      visionMeasurementStdDevs = visionMeasurementStdDevsFar;
      distanceLimit = 0.1; //Want to have some ability to override bad odom and should have megatTag at this range for good accuracy
    }

    boolean validPoint = currentLimeLightPose.pose3d.getX() != 0 && currentLimeLightPose.pose3d.getY() != 0;

    if (currentLimeLightPose.timestamp != previousPipelineTimestamp && currentLimeLightPose.tagID != 0 
    && distancePrev <= distanceLimit && validPoint){
      previousPipelineTimestamp = currentLimeLightPose.timestamp;
      previousTrans = getCurrentPose().getTranslation();
      // var target = pipelineResult.getBestTarget();
      // var fiducialId = pose.tagID;
      // Get the tag pose from field layout - consider that the layout will be null if
      // it failed to load
      // Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty()
      //     : aprilTagFieldLayout.getTagPose(fiducialId);
      // // if (fiducialId >= 0 && tagPose.isPresent()) {
      // var targetPose = tagPose.get();
      // // Transform3d camToTarget = target.getBestCameraToTarget();
      // // Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
      // // var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
      // var visionMeasurement = pose.pose3d;

      // SmartDashboard.putNumber("Timestamp", pose.timestamp);
      // //poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(),
      // pose.timestamp);
      // }
      // SmartDashboard.putNumber("PoseX", pose.pose3d.toPose2d().getX());
      // SmartDashboard.putNumber("PoseY", pose.pose3d.toPose2d().getY());
      // SmartDashboard.putNumber("PoseRot", pose.pose3d.toPose2d().getRotation().getDegrees());
      // SmartDashboard.putNumber("Pose TS", pose.timestamp);
      poseEstimator.addVisionMeasurement(currentLimeLightPose.pose3d.toPose2d(),
          currentLimeLightPose.timestamp);
    }

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  private String getFormattedLimePose() {
    var pose = currentLimeLightPose.pose3d.toPose2d();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        drive.getGyroRotation(),
        drive.getModulePositions(),
        newPose);
        previousTrans = newPose.getTranslation();
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public void setAllPoseToLimeLight(){
    setCurrentPose(currentLimeLightPose.pose3d.toPose2d());
    drive.resetOdometry(currentLimeLightPose.pose3d.toPose2d());
  }

  public double getTagDistance(){
    return tagDistance;
  }



}