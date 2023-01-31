// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private Translation3d botTran3d;
  private Rotation3d botRot3d;

  public LimeLightPoseEstimator(String limelightName) {
    table = NetworkTableInstance.getDefault().getTable(limelightName);
    poseSub = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    tidSub = table.getDoubleTopic("tid").subscribe(0.0);
  }

  public TimestampedBotPose3d[] getBotPose3d() {
    TimestampedDoubleArray[] result = poseSub.readQueue();
    int len = result.length;
    TimestampedBotPose3d[] timestampedPoses3D = new TimestampedBotPose3d[len];

    int i = 0;
    for (TimestampedDoubleArray poseValue : result) {
      timestampedPoses3D[i].timestamp = poseValue.timestamp;
      botTran3d = new Translation3d(poseValue.value[0], poseValue.value[1], poseValue.value[2]);
      botRot3d = new Rotation3d(poseValue.value[3], poseValue.value[4], poseValue.value[5]);
      timestampedPoses3D[i].pose3d = new Pose3d(botTran3d, botRot3d);
      i ++;
    }

    return timestampedPoses3D;
  }

}
