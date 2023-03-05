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
   * Creates a command that moves the {@link Arm} towards a given pose.
   *
   * @param arm The {@link Arm} to control
   * @param position The pose to move towards
   */
  public ScoringSystemTowardsPositionCommand(Arm arm, ArmPose position) {
    this(arm, () -> position);
  }

  /**
   * Creates a command that moves the {@link Arm} towards a given pose.
   *
   * @param arm The {@link Arm} to control
   * @param pose A supplier for the pose to move towards
   */
  public ScoringSystemTowardsPositionCommand(Arm arm, Supplier<ArmPose> pose) {
    this.addCommands(new RunElevatorToExtensionCommand(arm, () -> pose.get().getExtension()),
        new RotatePivotToAngleCommand(arm, () -> pose.get().getAngle().getDegrees()));
    addRequirements(arm);
  }
}
