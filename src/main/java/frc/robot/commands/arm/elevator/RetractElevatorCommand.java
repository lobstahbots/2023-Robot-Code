// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RetractElevatorCommand extends CommandBase {
  private boolean needsToExtend;
  private final Elevator elevator;

  /**
   * Creates a command that fully retracts the {@link Elevator} until the limit switch is flipped. If the limit switch
   * is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct for any
   * partial triggering.
   * 
   * @param elevator The {@link Elevator} to control
   */
  public RetractElevatorCommand(Elevator elevator) {
    this.elevator = elevator;
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
        elevator.move(ElevatorConstants.HOME_SPEED);
      } else {
        this.needsToExtend = false;
      }
    } else {
      if (!elevator.isRetracted()) {
        elevator.moveToLimitSwitch();
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
