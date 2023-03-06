// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.arm.Elevator;

public class RetractElevatorCommand extends CommandBase {
  protected boolean needsToExtend;
  protected final Elevator elevator;

  /**
   * Creates a command that fully retracts the {@link Arm.Elevator} until the limit switch is flipped. If the limit
   * switch is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct
   * for any partial triggering.
   * 
   * @param arm The {@link Arm} to control
   */
  public RetractElevatorCommand(Arm arm) {
    this.elevator = arm.getElevator();
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    this.needsToExtend = elevator.isRetracted();
    SmartDashboard.putBoolean("Is Retracted Already", elevator.isRetracted());
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
    return false;
  }
}
