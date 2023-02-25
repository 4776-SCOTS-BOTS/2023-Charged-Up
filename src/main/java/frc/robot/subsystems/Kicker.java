// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.PneumaticsConstants;
public class Kicker extends SubsystemBase {
  /** Creates a new Kicker. */
  private Solenoid kickerSolenoid;
  public Kicker(boolean kickerOut) {
    kickerSolenoid = new Solenoid(PneumaticsConstants.phCanID, PneumaticsModuleType.REVPH,PneumaticsConstants.kickerSolenoidPort);
    if(kickerOut){
      extendKicker();
    } else {
      retractKicker();
    }
  }
  public void extendKicker(){
    kickerSolenoid.set(true);
  }

  public void retractKicker(){
    kickerSolenoid.set(false);
  }
 

}
