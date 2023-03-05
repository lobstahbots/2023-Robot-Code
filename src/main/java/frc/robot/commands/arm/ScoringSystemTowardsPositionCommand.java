// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ArmPose;
import frc.robot.commands.arm.elevator.RunElevatorToExtensionCommand;
import frc.robot.commands.arm.pivot.RotatePivotToAngleCommand;
import frc.robot.subsystems.Arm;

public class ScoringSystemTowardsPositionCommand extends ParallelCommandGroup {
  /**
   * Creates a command that moves the {@link Arm} towards a given position.
   *
   * @param arm The {@link Arm} to control
   * @param position The position to move towards
   */
  public ScoringSystemTowardsPositionCommand(Arm arm, ArmPose position) {
    this(arm, () -> position);
  }

  /**
   * Creates a command that moves the {@link Arm} towards a given position.
   *
   * @param arm The {@link Arm} to control
   * @param position A supplier for the position to move towards
   */
  public ScoringSystemTowardsPositionCommand(Arm arm, Supplier<ArmPose> position) {
    this.addCommands(new RunElevatorToExtensionCommand(arm, () -> position.get().getElevatorExtension()),
        new RotatePivotToAngleCommand(arm, () -> position.get().getArmAngle().getDegrees()));
    addRequirements(arm);
  }
}
