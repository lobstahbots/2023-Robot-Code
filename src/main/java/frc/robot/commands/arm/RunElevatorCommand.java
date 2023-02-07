// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class RunElevatorCommand extends CommandBase {

  public final Elevator elevator;
  public final double speed;

  /**
   * Creates a command that extends/retracts the {@link Elevator} at a given speed.
   *
   * @param elevator The {@link Elevator} to control
   * @param speed The speed at which to extend/retract the elevator
   */
  public RunElevatorCommand(Elevator elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(this.elevator);
  }

  @Override
  public void execute() {
    elevator.extend(speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.extend(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
