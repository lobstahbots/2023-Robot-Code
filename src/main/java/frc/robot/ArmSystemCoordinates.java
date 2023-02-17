// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/** A class containing methods to convert between arm coordinates and Cartesian systems. */
public class ArmSystemCoordinates {

  public static double getArmDistanceToPositionArmCoordinates(Arm arm, Elevator elevator, Translation2d position) {
    return Math
        .abs(new Translation2d(elevator.getExtension(), new Rotation2d(Math.toRadians(arm.getAngle())))
            .getDistance(position));
  }

  public static double getArmDistanceToPositionCartesian(Arm arm, Elevator elevator, Translation2d position) {
    return getArmPositionCartesian(arm, elevator).getDistance(position);
  }

  public static Translation2d getArmPositionCartesian(Arm arm, Elevator elevator) {
    Rotation2d armAngle = new Rotation2d(Math.toRadians(30 + arm.getAngle()));
    Translation2d armPosition =
        new Translation2d(elevator.getLength(), armAngle);
    double xComponent = Math.abs(armPosition.getX()) + ArmConstants.PIVOT_SETBACK;
    double yComponent = ArmConstants.PIVOT_HEIGHT_FROM_GROUND;
    if (armAngle.getDegrees() > 90) {
      yComponent += armPosition.getY();
    } else {
      yComponent -= armPosition.getY();
    }

    return new Translation2d(xComponent, yComponent);
  }
}
