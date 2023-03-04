// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.subsystems.Arm;

public class RunElevatorToExtensionCommand extends CommandBase {
  private final Arm.Elevator elevator;
  private final DoubleSupplier extension;

  /**
   * Creates a command that runs the {@link Arm.Elevator} to a given extension length.
   *
   * @param arm The {@link Arm} to control
   * @param extension A supplier for the extension length to run the elevator to
   */
  public RunElevatorToExtensionCommand(Arm arm, DoubleSupplier extension) {
    this.elevator = arm.getElevator();
    this.extension = extension;

    addRequirements(arm);
  }

  /**
   * Creates a command that runs the {@link Arm.Elevator} to a given extension length.
   *
   * @param arm The {@link Arm} to control
   * @param extension The extension length to run the elevator to
   */
  public RunElevatorToExtensionCommand(Arm arm, double extension) {
    this(arm, () -> extension);
  }

  @Override
  public void initialize() {
    this.elevator.resetPID();
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
