// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.customClass;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor acting
 * against the force of gravity on a beam suspended at an angle).
 */
public class ShoulderFeedFowardController {
  public final double ks;
  public final double kgAlpha;
  public final double kgBeta;
  public final double kv;
  public final double ka;

  /**
   * Creates a new ArmFeedforward with the specified gains. Units of the gain values will dictate
   * units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   */
  public ShoulderFeedFowardController(double ks, double kgAlpha, double kgBeta, double kv, double ka) {
    this.ks = ks;
    this.kgAlpha = kgAlpha;
    this.kgBeta = kgBeta;
    this.kv = kv;
    this.ka = ka;
  }

  /**
   * Creates a new ArmFeedforward with the specified gains. Acceleration gain is defaulted to zero.
   * Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   */
  public ShoulderFeedFowardController(double ks, double kgAlpha, double kgBeta, double kv) {
    this(ks, kgAlpha, kgBeta, kv, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocityRadPerSec The velocity setpoint.
   * @param accelRadPerSecSquared The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(
      double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared, double elbowPositionRadians) {
    return ks * Math.signum(velocityRadPerSec)
        + kgAlpha * Math.cos(positionRadians)
        - kgBeta * Math.cos(positionRadians + elbowPositionRadians)
        + kv * velocityRadPerSec
        + ka * accelRadPerSecSquared;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
   * zero).
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double positionRadians, double velocity, double elbowPositionRadians) {
    return calculate(positionRadians, velocity, 0, elbowPositionRadians);
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

//   /**
//    * Calculates the maximum achievable velocity given a maximum voltage supply, a position, and an
//    * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
//    * profile are simultaneously achievable - enter the acceleration constraint, and this will give
//    * you a simultaneously-achievable velocity constraint.
//    *
//    * @param maxVoltage The maximum voltage that can be supplied to the arm.
//    * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
//    *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
//    *     not follow this convention, an offset should be added.
//    * @param acceleration The acceleration of the arm.
//    * @return The maximum possible velocity at the given acceleration and angle.
//    */
//   public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
//     // Assume max velocity is positive
//     return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka) / kv;
//   }

//   /**
//    * Calculates the minimum achievable velocity given a maximum voltage supply, a position, and an
//    * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
//    * profile are simultaneously achievable - enter the acceleration constraint, and this will give
//    * you a simultaneously-achievable velocity constraint.
//    *
//    * @param maxVoltage The maximum voltage that can be supplied to the arm.
//    * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
//    *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
//    *     not follow this convention, an offset should be added.
//    * @param acceleration The acceleration of the arm.
//    * @return The minimum possible velocity at the given acceleration and angle.
//    */
//   public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
//     // Assume min velocity is negative, ks flips sign
//     return (-maxVoltage + ks - Math.cos(angle) * kg - acceleration * ka) / kv;
//   }

//   /**
//    * Calculates the maximum achievable acceleration given a maximum voltage supply, a position, and
//    * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
//    * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
//    * simultaneously-achievable acceleration constraint.
//    *
//    * @param maxVoltage The maximum voltage that can be supplied to the arm.
//    * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
//    *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
//    *     not follow this convention, an offset should be added.
//    * @param velocity The velocity of the arm.
//    * @return The maximum possible acceleration at the given velocity.
//    */
//   public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
//     return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg - velocity * kv) / ka;
//   }

//   /**
//    * Calculates the minimum achievable acceleration given a maximum voltage supply, a position, and
//    * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
//    * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
//    * simultaneously-achievable acceleration constraint.
//    *
//    * @param maxVoltage The maximum voltage that can be supplied to the arm.
//    * @param angle The angle of the arm. This angle should be measured from the horizontal (i.e. if
//    *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does
//    *     not follow this convention, an offset should be added.
//    * @param velocity The velocity of the arm.
//    * @return The minimum possible acceleration at the given velocity.
//    */
//   public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
//     return maxAchievableAcceleration(-maxVoltage, angle, velocity);
//   }
}