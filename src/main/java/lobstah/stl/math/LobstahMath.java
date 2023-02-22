// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** This class stores relevant methods for mathematical operations, conversions, and scaling. */
public class LobstahMath {
  public static double kFalconIntegratedEncoderResolution = 2048;

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
        positionMeters / (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES));
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
        wheelRotationsPerSecond * (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES));
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
        wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.RobotConstants.WHEEL_DIAMETER_INCHES));
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

  /**
   * Convert the linear velocity of an object to its angular velocity
   *
   * @param linearVelocity linear velocity in meters per second
   * @param radius radius of object in meters
   * @return angular velocity of object in rad/s
   */
  public static double metersPerSecondToRadiansPerSecond(double linearVelocity, double radius) {
    return linearVelocity / radius;
  }

  /**
   * Convert the angular velocity of an object to its linear velocity
   *
   * @param angularVelocity angular velocity in rad/s
   * @param radius radius of rotating object in meters
   * @return linear velocity of object in meters per second
   */
  public static double radiansPerSecondToMetersPerSecond(double angularVelocity, double radius) {
    return angularVelocity * radius;
  }

  /**
   * Convert the RPM of a rotating object to its linear velocity
   *
   * @param rpm rotations per minute of object
   * @param radius radius of object in meters
   * @return Linear velocity in meters per second
   */
  public static double rotationsPerMinuteToMetersPerSecond(double rpm, double radius) {
    return radiansPerSecondToMetersPerSecond(Units.rotationsPerMinuteToRadiansPerSecond(rpm), radius);
  }

  /**
   * Convert the angular velocity of a rotating object to its RPM
   *
   * @param linearVelocity linear velocity of rotating object
   * @param radius radius of rotating object
   * @return RPM of object
   */
  public static double metersPerSecondToRotationPerMinute(double linearVelocity, double radius) {
    return Units.radiansPerSecondToRotationsPerMinute(
        metersPerSecondToRadiansPerSecond(linearVelocity, radius));
  }

  /**
   * Convert Falcon500 integrated encoder velocity to angular velocity
   *
   * @param falconVelocity velocity reported by Falcon500. ticks per 100ms
   * @param gearRatio gear ratio between motor and rotating object. Number of input rotations / number of output
   *          rotations
   * @return angular velocity of rotating object in rad/s
   */
  public static double falcon500VelocityToRadiansPerSecond(
      double falconVelocity, double gearRatio) {
    return Units.rotationsPerMinuteToRadiansPerSecond(
        falcon500VelocityToRotationsPerMinute(falconVelocity))
        / gearRatio;
  }

  /**
   * Convert Falcon500 integrated encoder velocity to angular velocity. Assumes there is no gearbox
   *
   * @param falconVelocity velocity reported by Falcon500. ticks per 100ms
   * @return angular velocity of rotating object in rad/s
   */
  public static double falcon500VelocityToRadiansPerSecond(double falconVelocity) {
    return falcon500VelocityToRadiansPerSecond(falconVelocity, 1);
  }

  /**
   * Convert angular velocity of an object to Falcon500 native velocity units, ticks per 100ms
   *
   * @param angularVelocity angular velocity of object in rad/s
   * @param gearRatio gear ratio between motor and rotating object. Number of input rotations / number of output
   *          rotations
   * @return Falcon500 native velocity units, ticks per 100ms
   */
  public static double radiansPerSecondToFalcon500Velocity(
      double angularVelocity, double gearRatio) {
    return rotationsPerMinuteToFalcon500Velocity(
        Units.radiansPerSecondToRotationsPerMinute(angularVelocity))
        * gearRatio;
  }

  /**
   * Convert angular velocity of an object to Falcon500 native velocity units, ticks per 100ms. Assumes there is no
   * gearbox
   *
   * @param angularVelocity angular velocity of object in rad/s
   * @return Falcon500 native velocity units, ticks per 100ms
   */
  public static double radiansPerSecondToFalcon500Velocity(double angularVelocity) {
    return radiansPerSecondToFalcon500Velocity(angularVelocity, 1);
  }

  /**
   * Convert Falcon500 native velocity units, ticks per 100ms to linear velocity
   *
   * @param falconVelocity linear velocity in meters per second
   * @param radius radius of rotating object in meters
   * @param gearRatio gear ratio between motor and rotating object. Number of input rotations / number of output
   *          rotations
   * @return linear velocity of rotating object in meters per second
   */
  public static double falcon500VelocityToMetersPerSecond(
      double falconVelocity, double radius, double gearRatio) {
    return rotationsPerMinuteToMetersPerSecond(
        falcon500VelocityToRotationsPerMinute(falconVelocity), radius)
        / gearRatio;
  }

  /**
   * Convert Falcon500 native velocity units, ticks per 100ms to linear velocity. Assumes there is no gearbox
   *
   * @param falconVelocity linear velocity in meters per second
   * @param radius radius of rotating object in meters
   * @return linear velocity of rotating object in meters per second
   */
  public static double falcon500VelocityToMetersPerSecond(double falconVelocity, double radius) {
    return falcon500VelocityToMetersPerSecond(falconVelocity, radius, 1);
  }

  /**
   * Convert linear velocity to Falcon500 native velocity units, ticks per 100ms.
   *
   * @param linearVelocity of object in meters per second
   * @param radius radius of rotating object
   * @param gearRatio gear ratio between motor and rotating object. Number of input rotations / number of output
   *          rotations
   * @return Falcon500 native velocity units, ticks per 100ms
   */
  public static double metersPerSecondToFalcon500Velocity(
      double linearVelocity, double radius, double gearRatio) {
    return rotationsPerMinuteToFalcon500Velocity(
        metersPerSecondToRotationPerMinute(linearVelocity, radius))
        * gearRatio;
  }

  /**
   * Convert linear velocity to Falcon500 native velocity units, ticks per 100ms. Assumes there is no gearbox
   *
   * @param linearVelocity of object in meters per second
   * @param radius radius of rotating object
   * @return Falcon500 native velocity units, ticks per 100ms
   */
  public static double metersPerSecondToFalcon500Velocity(double linearVelocity, double radius) {
    return metersPerSecondToFalcon500Velocity(linearVelocity, radius, 1);
  }

  /**
   * Convert Falcon500 native velocity units, ticks per 100ms, to RPM
   *
   * @param falconVelocity Falcon500 native velocity units, ticks per 100ms, of rotating object
   * @return rotations per minute
   */
  public static double falcon500VelocityToRotationsPerMinute(double falconVelocity) {
    return falconVelocity * 600.0 / kFalconIntegratedEncoderResolution;
  }

  /**
   * Convert RPM to Falcon500 native velocity units, ticks per 100ms
   *
   * @param rpm rotations per minute
   * @return Falcon500 native velocity units, ticks per 100ms, of rotating object
   */
  public static double rotationsPerMinuteToFalcon500Velocity(double rpm) {
    return rpm * kFalconIntegratedEncoderResolution / 600.0;
  }

  /**
   * Convert the position reported by the Falcon500 integrated encoder position to degrees
   *
   * @param falconPosition Falcon500 native position units
   * @return Encoder position in degrees
   */
  public static double falcon500PositionToDegrees(double falconPosition) {
    return (falconPosition / kFalconIntegratedEncoderResolution) * 360;
  }

  /**
   * Convert the position reported by the Falcon500 integrated encoder position to radians
   *
   * @param falconPosition Falcon500 native position units
   * @return Encoder position in radians
   */
  public static double falcon500PositionToRadians(double falconPosition) {
    return Math.toRadians(falcon500PositionToDegrees(falconPosition));
  }

  /**
   * Convert position in degrees to Falcon500 native position units
   *
   * @param positionDegrees position in degrees
   * @return Falcon500 native position units
   */
  public static double degreesToFalcon500Position(double positionDegrees) {
    double optimizedPosition = positionDegrees % 360;
    return (optimizedPosition / 360.0) * kFalconIntegratedEncoderResolution;
  }

  /**
   * Convert position in radians to Falcon500 native position units
   *
   * @param positionRadians position in radians
   * @return Falcon500 native position units
   */
  public static double radiansToFalcon500Position(double positionRadians) {
    return degreesToFalcon500Position(Math.toDegrees(positionRadians));
  }
}
