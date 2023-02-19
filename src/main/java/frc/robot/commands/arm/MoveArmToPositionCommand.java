// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmSystemCoordinates;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.arm.elevator.ResetElevatorCommand;
import frc.robot.commands.arm.elevator.RunElevatorToLengthCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class MoveArmToPositionCommand extends SequentialCommandGroup {
  /**
   * Creates a command that rotates the {@link Arm} and extends the {@link Elevator} to a given position.
   *
   * @param arm The {@link Arm} to control
   * @param elevator The {@link Elevator} to control
   * @param position The position to rotate the arm to
   */
  public MoveArmToPositionCommand(Arm arm, Elevator elevator, Translation2d finalPosition) {
    Translation2d polarPosition = ArmSystemCoordinates.getPolarPosition(finalPosition);

    double targetRotation = polarPosition.getAngle().getDegrees();
    double targetExtension = polarPosition.getNorm() - ElevatorConstants.LENGTH_FULLY_RETRACTED;

    addCommands(new ResetElevatorCommand(elevator)
        .unless(() -> Math.abs(arm.getRotation() - targetRotation) < ArmConstants.RETRACT_BEFORE_MOVING_DEADBAND),
        new RotateArmToAngleCommand(arm, targetRotation)
            .until(
                () -> Math.abs(targetRotation - arm.getRotation()) < ArmConstants.SEQUENTIAL_ROTATION_ERROR_DEADBAND),

        new ParallelCommandGroup(
            new RotateArmToAngleCommand(arm, targetRotation),
            new RunElevatorToLengthCommand(elevator, targetExtension)));

    addRequirements(arm, elevator);
  }
}
