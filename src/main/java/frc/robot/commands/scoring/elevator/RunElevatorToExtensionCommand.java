// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoringSystemConstants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RunElevatorToExtensionCommand extends CommandBase {
  private final Elevator elevator;
  private final DoubleSupplier extension;

  /**
   * Creates a command that runs the {@link Elevator} to a given extension length.
   *
   * @param elevator The {@link Elevator} to control
   * @param extension A supplier for the extension length to run the elevator to
   */
  public RunElevatorToExtensionCommand(Elevator elevator, DoubleSupplier extension) {
    this.elevator = elevator;
    this.extension = extension;

    addRequirements(this.elevator);
  }

  /**
   * Creates a command that runs the {@link Elevator} to a given extension length.
   *
   * @param elevator The {@link Elevator} to control
   * @param extension The extension length to run the elevator to
   */
  public RunElevatorToExtensionCommand(Elevator elevator, double extension) {
    this(elevator, () -> extension);
  }

  @Override
  public void execute() {
    double clampedExtension = MathUtil.clamp(extension.getAsDouble(), ElevatorConstants.MIN_EXTENSION_INCHES,
        ElevatorConstants.MAX_EXTENSION_INCHES);
    this.elevator.setPIDGoal(clampedExtension);
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
