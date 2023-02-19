// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** This class stores relevant methods for mathematical operations, conversions, and scaling. */
public class LobstahMath {

  /**
   * Scales a number on a range of values to a corresponding value on a different range
   * 
   * @param x The number to scale.
   * @param inputMin The original range's lower bound
   * @param inputMax The original range's upper bound
   * @param outputMin The new range's lower bound
   * @param outputMax The new range's upper bound
   */
  public static double scaleNumberToRange(double x, double inputMin, double inputMax, double outputMin,
      double outputMax) {
    double inputRange = inputMax - inputMin;
    double outputRange = outputMax - outputMin;

    if (inputRange == 0)
      throw new IllegalArgumentException("Input range cannot be 0");

    return ((x - inputMin) / inputRange * outputRange) + outputMin;
  }

  public static double scaleNumberToClampedRange(double x, double inputMin, double inputMax, double outputMin,
      double outputMax) {
    x = MathUtil.clamp(x, inputMin, outputMax);
    return scaleNumberToRange(x, inputMin, inputMax, outputMin, outputMax);
  }

  /**
   * Converts a distance in meters to native Falcon units of sensor counts.
   * 
   * @param positionMeters The distance in meters to convert to native units
   */
  public static int distanceToNativeUnits(double positionMeters) {
    double wheelRotations =
        positionMeters / (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.WHEEL_RADIUS_INCHES));
    double motorRotations = wheelRotations * Constants.RobotConstants.SENSOR_GEAR_RATIO;
    int sensorCounts = (int) (motorRotations * Constants.RobotConstants.COUNTS_PER_REV);
    return sensorCounts;
  }

  /**
   * Converts native Falcon velocity to meters per second.
   * 
   * @param nativeVelocity The native velocity to convert to meters per second.
   */
  public static double nativeUnitsToVelocityMetersPerSecond(double nativeVelocity) {
    double motorRotationsPer100ms = nativeVelocity / Constants.RobotConstants.COUNTS_PER_REV;
    double motorRotationsPerSecond = motorRotationsPer100ms * 10;
    double wheelRotationsPerSecond = motorRotationsPerSecond / Constants.RobotConstants.SENSOR_GEAR_RATIO;
    double velocityMetersPerSecond =
        wheelRotationsPerSecond * (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.WHEEL_RADIUS_INCHES));
    return velocityMetersPerSecond;
  }

  /**
   * Converts a distance native Falcon units of sensor counts to meters per second.
   * 
   * @param positionMeters The number of sensor counts to convert to meters.
   */
  public static double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / Constants.RobotConstants.COUNTS_PER_REV;
    double wheelRotations = motorRotations / Constants.RobotConstants.SENSOR_GEAR_RATIO;
    double positionMeters =
        wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.WHEEL_RADIUS_INCHES));
    return positionMeters;
  }

  /**
   * Calculates turning output based on current and desired angle, for gyro values clamped between 180 and -180 degrees.
   * 
   * @param currentAngle The current gyro heading in degrees, 180 to -180.
   * @param desiredAngle The desired gyro heading in degrees, 180 to -180.
   */
  public static double calculateTurningOutput(double currentAngle, double desiredAngle) {
    double output = currentAngle - desiredAngle;
    output %= 360;
    if (Math.abs(output) > 180)
      output -= Math.signum(output) * 360;
    return output;
  }

}
