// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.subsystems.Arm;

public class RunElevatorCommand extends CommandBase {

  public final Arm.Elevator elevator;
  public final DoubleSupplier speed;


  /**
   * Creates a command that extends/retracts the {@link Arm.Elevator} at the speed given by the supplier.
   *
   * @param arm The {@link Arm} to control
   * @param speed Supplier for the speed at which to extend/retract the elevator
   */
  public RunElevatorCommand(Arm arm, DoubleSupplier speed) {
    this.elevator = arm.getElevator();
    this.speed = speed;
    addRequirements(arm);
  }

  /**
   * Creates a command that extends/retracts the {@link Arm.Elevator} at a given speed.
   *
   * @param arm The {@link Arm} to control
   * @param speed The speed at which to extend/retract the elevator
   */
  public RunElevatorCommand(Arm arm, double speed) {
    this(arm, () -> speed);
  }

  @Override
  public void execute() {
    if ((elevator.getExtension() > ElevatorConstants.MAX_EXTENSION_INCHES && speed.getAsDouble() > 0)
        || (elevator.getExtension() < ElevatorConstants.MIN_EXTENSION_INCHES && speed.getAsDouble() < 0)) {
      elevator.move(0.0);
    } else {
      elevator.move(speed.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.move(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

