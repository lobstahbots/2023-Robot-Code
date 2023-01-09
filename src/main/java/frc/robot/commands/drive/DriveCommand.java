// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

/**
 * A command used to control a {@link DriveBase}.
 */
public abstract class DriveCommand extends CommandBase {
  protected final DriveBase driveBase;

  /**
   * Creates a command used to control a {@link DriveBase}.
   *
   * @param driveBase The {@link DriveBase} to control
   */
  public DriveCommand(DriveBase driveBase) {
    addRequirements(driveBase);
    this.driveBase = driveBase;
  }
}
