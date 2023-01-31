// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.customClass;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class TimestampedBotPose3d {
    public double timestamp;
    public Pose3d pose3d;
    public int tagID;

    public TimestampedBotPose3d() {
        this.timestamp = 0;
        this.pose3d = new Pose3d();
        this.tagID = 0;
    }

    public TimestampedBotPose3d(double timestamp, Pose3d pose3d, int tagID) {
        this.timestamp = timestamp;
        this.pose3d = pose3d;
        this.tagID = tagID;
    }
}
