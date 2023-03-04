// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.customClass.TimestampedBotPose3d;

public class LimeLightPoseEstimator extends SubsystemBase {
  private NetworkTable table;
  private DoubleArraySubscriber poseSub;
  private DoubleSubscriber tidSub;
  private DriverStation.Alliance alliance = Alliance.Invalid;

  private Translation3d botTran3d;
  private Rotation3d botRot3d;

  public LimeLightPoseEstimator(String limelightName) {
    String botPoseName;

    this.alliance = DriverStation.getAlliance();
    if (this.alliance == Alliance.Red) {
      botPoseName = "botpose_wpired";
    } else {
      this.alliance = Alliance.Blue;
      botPoseName = "botpose_wpiblue";
    }

    table = NetworkTableInstance.getDefault().getTable(limelightName);
    poseSub = table.getDoubleArrayTopic(botPoseName).subscribe(new double[] {});
    tidSub = table.getDoubleTopic("tid").subscribe(0.0);
  }

  public TimestampedBotPose3d[] getBotPose3d() {
    // Not using multiple poses, but keeping the array version in case we want to
    double[] result = poseSub.get(); // This would change to .get
    TimestampedBotPose3d[] timestampedPoses3D = new TimestampedBotPose3d[1];
    int tagID = getAprilTagID();
    if (tagID != 0) {

      int len = result.length; // Not needed for single reading

      // timestampedPoses3D[0] = new TimestampedBotPose3d();

      double timestamp = Timer.getFPGATimestamp() - (result[6] / 1000.0);
      Translation3d botTran3d = new Translation3d(result[0], result[1], result[2]);
      Rotation3d botRot3d = new Rotation3d(Math.toRadians(result[3]), Math.toRadians(result[4]), Math.toRadians(result[5]));
      Pose3d pose3d = new Pose3d(botTran3d, botRot3d);
      // timestampedPoses3D[0].tagID = getAprilTagID();

      timestampedPoses3D[0] = new TimestampedBotPose3d(timestamp, pose3d, tagID);
    } else {
      timestampedPoses3D[0] = new TimestampedBotPose3d();
    }

    // Array based code
    // int i = 0;
    // for (double poseValue : result) {
    // timestampedPoses3D[i].timestamp = Timer.getFPGATimestamp() -
    // (poseValue[6]/1000.0);
    // botTran3d = new Translation3d(poseValue.value[0], poseValue.value[1],
    // poseValue.value[2]);
    // botRot3d = new Rotation3d(poseValue.value[3], poseValue.value[4],
    // poseValue.value[5]);
    // timestampedPoses3D[i].pose3d = new Pose3d(botTran3d, botRot3d);
    // i ++;
    // }

    return timestampedPoses3D;
  }

  public int getAprilTagID() {
    return (int) tidSub.get();
  }

}
