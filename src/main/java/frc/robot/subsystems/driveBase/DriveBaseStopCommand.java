// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Repeatedly sets the {@link DriveBase} speed to 0.
 */
public class DriveBaseStopCommand extends CommandBase {
  private final DriveBase driveBase;

  /**
   * Creates a command that repeatedly sets the {@link DriveBase} speed to 0.
   *
   * @param driveBase The {@link DriveBase} to control
   */
  public DriveBaseStopCommand(DriveBase driveBase) {
    this.driveBase = driveBase;
    addRequirements(driveBase);
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
