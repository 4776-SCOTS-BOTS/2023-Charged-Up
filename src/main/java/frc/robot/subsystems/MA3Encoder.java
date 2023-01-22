// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.Rotation2d;


public class MA3Encoder {
  private final AnalogInput mAnalogInput;

  private Rotation2d rotation = new Rotation2d();
  private Rotation2d home = new Rotation2d();
  private boolean reversed = false;
  
  /** Creates a new MA3Encoder. */
  public MA3Encoder(int port, Rotation2d homeLoc) {
    mAnalogInput = new AnalogInput(port);
    this.home = homeLoc;
  }

  public Rotation2d getCalbratedAngle() {
    return home.rotateBy(getRawAngle());
  }

  public Rotation2d getRawAngle(){
    rotation = new Rotation2d((reversed ? -1:1) * 2 * Math.PI * mAnalogInput.getVoltage() / 5.0); 
    return rotation;
  }

  public double get(){
    return getCalbratedAngle().getRadians();
  }

  public void setReverseDirection(boolean reversed){
    this.reversed = reversed;
  }

  public double getRawVolts() {
    return mAnalogInput.getVoltage();
  }
}
