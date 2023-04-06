// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.customClass.CRGB;


public class LED extends SubsystemBase {
  /** Creates a new LED. */
  // PWM port 9
    // Must be a PWM header, not MXP or DIO

   private AddressableLED m_led = new AddressableLED(Constants.LEDConstants.kLEDPort);
    private AddressableLEDBuffer  m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kStrandLength);  


  public LED(){
        // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
   m_led.setLength(m_ledBuffer.getLength());   
  }

  public void setDisplay () {
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setColor(CRGB color){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      // Light strand is RBG
      m_ledBuffer.setRGB(i, color.r, color.b, color.g);
   }
   
  }
  public void setYellow(){
    setColor(Constants.kRGB_yellow);
    setDisplay();
  }
  public void setPurple(){
    setColor(Constants.kRGB_purple);
    setDisplay();
  }
  public void setBlack(){
    setColor(Constants.kRGB_black);
    setDisplay();
  }
  
}
