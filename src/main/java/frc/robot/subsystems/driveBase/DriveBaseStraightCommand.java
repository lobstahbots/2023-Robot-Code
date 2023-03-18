// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driveBase;

import java.util.function.DoubleSupplier;

/**
 * Drives a {@link DriveBase} in a (roughly) straight line.
 */
public class DriveBaseStraightCommand extends DriveBaseTankCommand {
  /**
   * Drives the driveBase in a (roughly) straight line.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param speedSupplier Supplier for speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public DriveBaseStraightCommand(DriveBase driveBase,
      DoubleSupplier speedSupplier, boolean squaredInputs) {
    super(driveBase, speedSupplier, speedSupplier, squaredInputs);
  }

  /**
   * Drives the driveBase in a (roughly) straight line.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param speedSupplier Supplier for speed
   */
  public DriveBaseStraightCommand(DriveBase driveBase,
      DoubleSupplier speedSupplier) {
    super(driveBase, speedSupplier, speedSupplier, false);
  }

  /**
   * Drives the driveBase in a (roughly) straight line.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param leftSpeed The left speed
   * @param rightSpeed The right speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public DriveBaseStraightCommand(DriveBase driveBase, double speed, boolean squaredInputs) {
    this(driveBase, () -> speed, squaredInputs);
  }

  /**
   * Drives the driveBase in a (roughly) straight line.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param leftSpeed The left speed
   * @param rightSpeed The right speed
   */
  public DriveBaseStraightCommand(DriveBase driveBase, double speed) {
    this(driveBase, () -> speed, false);
  }
}
