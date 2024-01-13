// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public CANSparkMax motor1, motor2;
  public double shooterPwr = 0.9;
  public double shooterPwr2 = 0.9;

  /** Creates a new Shooter. */
  public Shooter(int can1, int can2) {
    motor1 = new CANSparkMax(can1, MotorType.kBrushless);
    motor2 = new CANSparkMax(can2, MotorType.kBrushless);

    motor2.setInverted(true);
    //motor2.follow(motor1, true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void launch(double power){
    motor1.set(power);
    motor2.set(power);
  }

  public void launch(){
    launch(shooterPwr);
  }

  public void launch(double power1, double power2) {
    motor1.set(power1);
    motor2.set(power2);
  }

  public void spinLaunch() {
    launch(shooterPwr, shooterPwr2);
  }

  public void stop(){
    motor1.set(0);
    motor2.set(0);
  }

}
