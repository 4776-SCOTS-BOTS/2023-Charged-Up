// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.customClass.TimestampedBotPose3d;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LimeLightCubeTracker extends SubsystemBase {
  private NetworkTable table;
  private DoubleSubscriber txSub, tySub, tvSub;

  //Constants for centerline measured from limelight
  private static final double X1 = 0;
  private static final double Y1 = 0;
  private static final double X2 = 2.2;
  private static final double Y2 = 8.7;

  private Timer timer = new Timer();
  private double lastGoodTime = -1;
  private double lastOnline = 1e6;

  DataLog log = DataLogManager.getLog();
  
  public LimeLightCubeTracker(String limelightName) {
    table = NetworkTableInstance.getDefault().getTable(limelightName);

    txSub = table.getDoubleTopic("tx").subscribe(0.0);
    tySub = table.getDoubleTopic("ty").subscribe(0.0);
    tvSub = table.getDoubleTopic("tv").subscribe(0.0);
  }

  public double cubeFromCenterLine(){
    double online;

    if(tvSub.get() == 1){
      lastGoodTime = timer.getFPGATimestamp();
      double x3 = txSub.get();
      double y3 = tySub.get();

      //Negative value means cone is to the right
      online = ( (X2-X1) * (y3-Y1) ) - ( (Y2 - Y1) * (x3-X1) );
    } else if (timer.getFPGATimestamp() - lastGoodTime < 0.25){
      online = lastOnline;
    } else {
      online = 1e6;
    }

    return online;
  }

}
