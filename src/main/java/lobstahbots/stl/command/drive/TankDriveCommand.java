// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import frc.robot.subsystems.DriveBase;

/**
 * Drives a {@link DriveBase} using tank drive controls.
 */
public class TankDriveCommand extends DriveCommand {

  private final Supplier<Double> leftSpeedSupplier;
  private final Supplier<Double> rightSpeedSupplier;
  private final boolean squaredInputs;

  /**
   * Drives the driveBase at the left and right speeds returned by their respective Suppliers.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param leftSpeedSupplier Supplier for left speed
   * @param rightSpeedSupplier Supplier for right speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public TankDriveCommand(DriveBase driveBase, Supplier<Double> leftSpeedSupplier,
      Supplier<Double> rightSpeedSupplier, boolean squaredInputs) {
    super(driveBase);
    this.leftSpeedSupplier = leftSpeedSupplier;
    this.rightSpeedSupplier = rightSpeedSupplier;
    this.squaredInputs = squaredInputs;
  }

  /**
   * Drives the driveBase at the given left and right speeds.
   *
   * @param driveBase The {@link DriveBase} to drive
   * @param leftSpeed The left speed
   * @param rightSpeed The right speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public TankDriveCommand(DriveBase driveBase, double leftSpeed, double rightSpeed, boolean squaredInputs) {
    this(driveBase, () -> leftSpeed, () -> rightSpeed, squaredInputs);
  }

  @Override
  public void execute() {
    driveBase.tankDrive(leftSpeedSupplier.get(), rightSpeedSupplier.get(), squaredInputs);
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
