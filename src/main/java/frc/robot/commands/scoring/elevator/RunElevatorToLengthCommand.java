// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RunElevatorToLengthCommand extends CommandBase {
  private final Elevator elevator;
  private final double position;

  /**
   * Creates a command that runs the {@link Elevator} to a given position.
   *
   * @param elevator The {@link Elevator} to control
   * @param position The position to run the elevator to
   */
  public RunElevatorToLengthCommand(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position =
        MathUtil.clamp(position, ElevatorConstants.MIN_EXTENSION_INCHES, ElevatorConstants.MAX_EXTENSION_INCHES);
    addRequirements(this.elevator);
  }

  @Override
  public void initialize() {
    this.elevator.setPIDGoal(this.position);
  }

  @Override
  public void execute() {
    this.elevator.feedPID();
  }

  @Override
  public void end(boolean interrupted) {
    elevator.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
