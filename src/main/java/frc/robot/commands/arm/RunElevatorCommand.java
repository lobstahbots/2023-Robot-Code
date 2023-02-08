// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class RunElevatorCommand extends CommandBase {

  public final Elevator elevator;
  public final Supplier<Double> speed;


  /**
   * Creates a command that extends/retracts the {@link Elevator} at the speed given by the supplier.
   *
   * @param elevator The {@link Elevator} to control
   * @param speed Supplier for the speed at which to extend/retract the elevator
   */
  public RunElevatorCommand(Elevator elevator, Supplier<Double> speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  /**
   * Creates a command that extends/retracts the {@link Elevator} at a given speed.
   *
   * @param elevator The {@link Elevator} to control
   * @param speed The speed at which to extend/retract the elevator
   */
  public RunElevatorCommand(Elevator elevator, double speed) {
    this(elevator, () -> speed);
  }

  @Override
  public void execute() {
    elevator.extend(speed.get());
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
