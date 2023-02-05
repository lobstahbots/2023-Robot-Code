// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateArmCommand extends CommandBase {
  private final Arm arm;
  private final int position;

  /**
   * Creates a command that rotates the {@link Arm} to a given position.
   *
   * @param arm The {@link Arm} to control
   * @param position The position to rotate the arm to
   */
  public RotateArmCommand(Arm arm, int position) {
    this.arm = arm;
    this.position = position;
  }

  public void initialize() {
    this.arm.enable();
  }

  public void execute() {
    this.arm.setGoal(this.position);
  }

  public boolean isFinished() {
    return false;
  }
}
