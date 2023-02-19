// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
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
    // gets the desired position in arm coordinates, arm (0,0) is at the pivot
    finalPosition = finalPosition.minus(new Translation2d(ArmConstants.PIVOT_SETBACK,
        ArmConstants.PIVOT_HEIGHT_FROM_GROUND - IntakeConstants.INTAKE_HEIGHT));

    double targetRotation = finalPosition.getAngle().plus(ArmConstants.ZERO_ARM_OFFSET).getDegrees();
    double targetExtension = finalPosition.getNorm() - ElevatorConstants.LENGTH_FULLY_RETRACTED;

    addCommands(new ResetElevatorCommand(elevator)
        .unless(() -> Math.abs(arm.getAngle() - targetRotation) < ArmConstants.RETRACT_BEFORE_MOVING_DEADBAND),
        new RotateArmToAngleCommand(arm, targetRotation)
            .until(() -> Math.abs(targetRotation - arm.getAngle()) < ArmConstants.SEQUENTIAL_ROTATION_ERROR_DEADBAND),
        // angle of the translation from the pivot to the target point is the angle the arm needs to rotate
        // however, the angle must be offset because the arm's 0-degree rotation is not actually vertical.
        new ParallelCommandGroup(
            new RotateArmToAngleCommand(arm, targetRotation),
            new RunElevatorToLengthCommand(elevator, targetExtension)));
    // length of the translation from the pivot to the target point is the total length the elevator needs to span;
    // need to subtract the fully retracted length of the elevator to calculate how much it needs to extend.);
    addRequirements(arm, elevator);
  }
}
