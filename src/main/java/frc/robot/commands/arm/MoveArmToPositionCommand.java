// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.arm.elevator.ResetElevatorCommand;
import frc.robot.commands.arm.elevator.RunElevatorToPositionCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class MoveArmToPositionCommand extends SequentialCommandGroup {
  private final Arm arm;
  private final Elevator elevator;
  private final Translation2d finalPosition;

  /**
   * Creates a command that rotates the {@link Arm} and extends the {@link Elevator} to a given position.
   *
   * @param arm The {@link Arm} to control
   * @param position The position to rotate the arm to
   */
  public MoveArmToPositionCommand(Arm arm, Elevator elevator, Translation2d finalPosition) {
    this.arm = arm;
    this.elevator = elevator;
    this.finalPosition =
        finalPosition.minus(new Translation2d(ArmConstants.PIVOT_SETBACK, ArmConstants.PIVOT_HEIGHT_FROM_GROUND));
    SmartDashboard.putString("Target Position", this.finalPosition.toString());
    addRequirements(this.arm, this.elevator);
    if (finalPosition.getX() < ArmPositionConstants.OUTSIDE_BUMPERS.getX()
        && finalPosition.getY() < ArmPositionConstants.OUTSIDE_BUMPERS.getY()) {
      addCommands(new ResetElevatorCommand(elevator, ElevatorConstants.RETRACT_SPEED),
          new RotateArmToAngleCommand(arm,
              ArmPositionConstants.OUTSIDE_BUMPERS.getAngle().minus(ArmConstants.ZERO_ARM_OFFSET).getDegrees()),
          new RunElevatorToPositionCommand(elevator,
              this.finalPosition.getNorm() - ElevatorConstants.LENGTH_FULLY_RETRACTED),
          new RotateArmToAngleCommand(arm,
              this.finalPosition.getAngle().minus(ArmConstants.ZERO_ARM_OFFSET).getDegrees()));
    } else {
      addCommands(new ResetElevatorCommand(elevator, ElevatorConstants.RETRACT_SPEED),
          new RotateArmToAngleCommand(arm,
              this.finalPosition.getAngle().minus(ArmConstants.ZERO_ARM_OFFSET).getDegrees()),
          new RunElevatorToPositionCommand(elevator,
              this.finalPosition.getNorm() - ElevatorConstants.LENGTH_FULLY_RETRACTED));
    }
  }
}
