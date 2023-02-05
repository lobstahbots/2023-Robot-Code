// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

/**
 * Toggles the {@link IdleMode} (aka Braking mode) on the given {@link DriveBase}.
 */
public class ToggleBrakingModeCommand extends CommandBase {
  private final DriveBase driveBase;

  /**
   * Toggles the {@link IdleMode} (aka Braking mode) on the given {@link DriveBase}.
   *
   * @param driveBase The {@link DriveBase} to control
   */
  public ToggleBrakingModeCommand(DriveBase driveBase) {
    this.driveBase = driveBase;
  }

  @Override
  public void initialize() {
    driveBase.toggleNeutralMode();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
