// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ScoringPosition;
import frc.robot.commands.scoring.arm.RotateArmToAngleCommand;
import frc.robot.commands.scoring.elevator.RunElevatorToExtensionCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoringSystemTowardsPositionCommand extends ParallelCommandGroup {
  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} towards a given position.
   *
   * @param arm The {@link Arm} to control
   * @param elevator The {@link Elevator} to control
   * @param position The position to move towards
   */
  public ScoringSystemTowardsPositionCommand(Arm arm, Elevator elevator, ScoringPosition position) {
    this(arm, elevator, () -> position);
  }

  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} towards a given position.
   *
   * @param arm The {@link Arm} to control
   * @param elevator The {@link Elevator} to control
   * @param position A supplier for the position to move towards
   */
  public ScoringSystemTowardsPositionCommand(Arm arm, Elevator elevator, Supplier<ScoringPosition> position) {
    this.addCommands(new RunElevatorToExtensionCommand(elevator, () -> position.get().getElevatorExtension()),
        new RotateArmToAngleCommand(arm, () -> position.get().getArmAngle().getDegrees()));
    addRequirements(arm, elevator);
  }
}
