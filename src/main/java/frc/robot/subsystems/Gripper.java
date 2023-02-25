// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.PneumaticsConstants;

public class Gripper extends SubsystemBase {
  private Solenoid gripSolenoid;
  
  /** Creates a new Gripper. */
  public Gripper(boolean gripClosed) {
    gripSolenoid = new Solenoid(PneumaticsConstants.phCanID, PneumaticsModuleType.REVPH,PneumaticsConstants.gripperSolenoidPort);
    if(gripClosed){
      closeGripper();
    } else {
      openGripper();
    }
  }
  
  
  /** Creates a new Gripper. */
  

  public void closeGripper(){
    gripSolenoid.set(false);
  }

  public void openGripper(){
    gripSolenoid.set(true);
  }
}
