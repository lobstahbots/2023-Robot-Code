// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmSystemCoordinates;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.subsystems.DriveBase;
import lobstah.stl.math.LobstahMath;

/* A command to set the acceleration limit of an {@link} DriveBase. */
public class SetAccelerationLimitCommand extends CommandBase {
  private final DriveBase driveBase;
  private final Supplier<Translation2d> armPosition;

  /**
   * Creates a command that repeatedly limits the {@link DriveBase} accleration based on the position of the arm.
   *
   * @param driveBase The {@link DriveBase} to control
   * @param armPosition A supplier that gives a {@link Translation2d} representing the current position of the arm in am
   *          polar coordinates.
   */
  public SetAccelerationLimitCommand(DriveBase driveBase, Supplier<Translation2d> armPosition) {
    this.driveBase = driveBase;
    this.armPosition = armPosition;
  }

  /**
   * Creates a command that repeatedly limits the {@link DriveBase} accleration based on the position of the arm.
   *
   * @param driveBase The {@link DriveBase} to control
   * @param armPosition A {@link Translation2d} that represents the current position of the arm in am polar coordinates.
   */
  public SetAccelerationLimitCommand(DriveBase driveBase, Translation2d armPosition) {
    this(driveBase, () -> armPosition);
  }

  @Override
  public void execute() {
    double rateLimit = 1 / LobstahMath.scaleNumberToClampedRange(
        ArmSystemCoordinates.getArmDistanceToPolarPosition(ArmPositionConstants.STOWED, armPosition.get()), 0,
        ArmPositionConstants.MAX_DISTANCE_EXTENDED,
        0, 3);
    driveBase.setRateLimit(rateLimit);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
