// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveBase;

/**
 * Turns a {@link DriveBase} to a provided angle.
 */
public class TurnCommand extends DriveCommand {
  private final double targetHeading;

  /**
   * Turns the driveBase to the target heading gyro value.
   *
   * @param driveBase The {@link DriveBase} to turn
   * @param targetHeading The desired angle in degrees. Negative for left, positive for right.
   */
  public TurnCommand(DriveBase driveBase, double targetHeading) {
    super(driveBase);
    this.targetHeading = targetHeading;
  }

  @Override
  public void execute() {
    double error = targetHeading - driveBase.getHeading().getDegrees();

    driveBase.tankDrive(DriveConstants.TURN_KP * error, -DriveConstants.TURN_KP * error, false);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(targetHeading - driveBase.getHeading().getDegrees()) < DriveConstants.TURN_DEADBAND) {
      return true;
    }

    return false;
  }
}
