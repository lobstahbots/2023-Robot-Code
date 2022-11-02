// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveBase;

/**
 * Repeatedly sets the {@link DriveBase} speed to 0.
 */
public class StopDriveCommand extends DriveCommand {

  /**
   * Creates a command that repeatedly sets the {@link DriveBase} speed to 0.
   *
   * @param driveBase The {@link DriveBase} to control
   */
  public StopDriveCommand(DriveBase driveBase) {
    super(driveBase);
  }

  @Override
  public void execute() {
    driveBase.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
