// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.customClass;

/** Add your docs here. */
public class ArmPosition {
    public double elbowRadians, shoulderRadians, elbowDegrees, shoulderDegrees;

    public ArmPosition(double elbowDegrees, double shoulderDegrees){
        this.elbowDegrees = elbowDegrees;
        this.shoulderDegrees = shoulderDegrees;
        this.elbowRadians = Math.toRadians(elbowDegrees);
        this.shoulderRadians = Math.toRadians(shoulderDegrees);

    }
}
