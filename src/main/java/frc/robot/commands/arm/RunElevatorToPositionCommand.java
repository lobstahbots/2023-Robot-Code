// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class RunElevatorToPositionCommand extends CommandBase {
  private final Elevator elevator;
  private final double position;

  /**
   * Creates a command that runs the {@link Elevator} to a given position.
   *
   * @param elevator The {@link Elevator} to control
   * @param position The position to run the elevator to
   */
  public RunElevatorToPositionCommand(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
  }

  public void execute() {
    this.elevator.setGoal(this.position);
  }

  public boolean isFinished() {
    return false;
  }
}
