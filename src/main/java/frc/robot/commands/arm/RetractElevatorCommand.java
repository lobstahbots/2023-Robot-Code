// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class RetractElevatorCommand extends CommandBase {
  private boolean needsToExtend;
  private final Elevator elevator;
  private final double speed;

  /**
   * Creates a command that fully retracts the {@link Elevator} until the limit switch is flipped. If the limit switch
   * is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct for any
   * partial triggering.
   * 
   * @param elevator The {@link Elevator} to control
   * @param speed The speed at which the elevator retracts
   */
  public RetractElevatorCommand(Elevator elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(this.elevator);
  }

  @Override
  public void initialize() {
    this.needsToExtend = elevator.isRetracted();
  }

  @Override
  public void execute() {
    if (this.needsToExtend) {
      if (elevator.isRetracted()) {
        elevator.move(-speed);
      } else {
        this.needsToExtend = false;
      }
    } else {
      if (!elevator.isRetracted()) {
        elevator.move(speed);
      } else {
        elevator.move(0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    // return !this.limitSwitchInitialValue && elevator.isRetracted();
    return false;
  }
}
