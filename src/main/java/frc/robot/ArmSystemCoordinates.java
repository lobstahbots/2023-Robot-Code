// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

/** A class containing methods to convert between arm coordinates and Cartesian systems. */
public class ArmSystemCoordinates {

  /**
   * Gets the distance between two {@link Translation2d} positions in arm polar coordinates.
   * 
   * @param armPosition The first position of the arm in arm polar coordinates
   * @param otherPosition The other position in arm polar coordinates to calculate distance to
   */
  public static double getArmDistanceToPolarPosition(Translation2d armPosition, Translation2d otherPosition) {
    return Math.abs(armPosition.getDistance(otherPosition));
  }

  /**
   * Gets the distance between a {@link Translation2d} position in arm polar coordinates and a {@link Translation2d}
   * position in Cartesian coordinates.
   * 
   * @param armPosition The position of the arm in arm polar coordinates
   * @param cartesianPosition The other position in Cartesian coordinates to calculate distance to
   */
  public static double getArmDistanceToCartesianPosition(Translation2d armPosition, Translation2d cartesianPosition) {
    return getArmDistanceToPolarPosition(armPosition, getPolarPosition(cartesianPosition));
  }

  /**
   * Converts a position in arm polar coordinates to Cartesian coordinates.
   * 
   * @param armPosition The position of the arm in arm polar coordinates
   */
  public static Translation2d getCartesianPosition(Translation2d armPosition) {
    armPosition =
        armPosition.plus(new Translation2d(0, ArmConstants.ANGLE_AT_ARM_ZERO));
    double xComponent = Math.abs(armPosition.getY()) + ArmConstants.PIVOT_SETBACK;
    double yComponent = ArmConstants.PIVOT_HEIGHT_FROM_GROUND - IntakeConstants.INTAKE_HEIGHT;
    if (armPosition.getAngle().getDegrees() > 90) {
      yComponent += Math.abs(armPosition.getX());
    } else {
      yComponent -= Math.abs(armPosition.getX());
    }
    return new Translation2d(xComponent, yComponent);
  }

  /**
   * Converts a position in Cartesian coordinates to arm polar coordinates.
   * 
   * @param cartesianPosition The position of the arm in Cartesian coordinates
   */
  public static Translation2d getPolarPosition(Translation2d cartesianPosition) {
    cartesianPosition = cartesianPosition.minus(new Translation2d(ArmConstants.PIVOT_SETBACK,
        ArmConstants.PIVOT_HEIGHT_FROM_GROUND - IntakeConstants.INTAKE_HEIGHT));
    // gets the desired position in arm coordinates, arm (0,0) is at the pivot

    Rotation2d offsetRotation = cartesianPosition.getAngle().plus(ArmConstants.ZERO_ARM_OFFSET);
    // angle of the translation from the pivot to the target point is the angle the arm needs to rotate
    // however, the angle must be offset because the arm's 0-degree rotation is not actually vertical.

    return new Translation2d(cartesianPosition.getNorm(), offsetRotation);
  }
}
