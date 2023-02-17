// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.arm.elevator.ResetElevatorCommand;
import frc.robot.commands.arm.elevator.RunElevatorToPositionCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class RotateArmToPositionCommand extends SequentialCommandGroup {
  private final Arm arm;
  private final Elevator elevator;
  private final Translation2d finalPosition;

  /**
   * Creates a command that rotates the {@link Arm} and extends the {@link Elevator} to a given position.
   *
   * @param arm The {@link Arm} to control
   * @param position The position to rotate the arm to
   */
  public RotateArmToPositionCommand(Arm arm, Elevator elevator, Translation2d finalPosition) {
    this.arm = arm;
    this.elevator = elevator;
    this.finalPosition =
        finalPosition.minus(new Translation2d(ArmConstants.PIVOT_SETBACK,
            ArmConstants.PIVOT_HEIGHT_FROM_GROUND - IntakeConstants.INTAKE_HEIGHT));
    // gets the desired position in arm coordinates, arm (0,0) is at the pivot
    SmartDashboard.putString("Target Arm Position", this.finalPosition.toString());
    SmartDashboard.putNumber("Desired offset with Angle",
        this.finalPosition.getAngle().plus(ArmConstants.ZERO_ARM_OFFSET).getDegrees());
    System.out.println("Running command");
    // if (finalPosition.getX() < ArmPositionConstants.OUTSIDE_BUMPERS.getX()
    // && finalPosition.getY() < ArmPositionConstants.OUTSIDE_BUMPERS.getY()) {
    // System.out.println("Inside bumpers");
    // // if the target position is within the collision point with the bumpers, the arm needs to first rotate to a
    // // position where it won't collide with them.
    // addCommands(new ResetElevatorCommand(elevator, ElevatorConstants.RETRACT_SPEED),
    // new RotateArmToAngleCommand(arm,
    // ArmPositionConstants.OUTSIDE_BUMPERS.getAngle().minus(ArmConstants.ZERO_ARM_OFFSET).getDegrees()),
    // new RunElevatorToPositionCommand(elevator,
    // this.finalPosition.getNorm() - ElevatorConstants.LENGTH_FULLY_RETRACTED),
    // new RotateArmToAngleCommand(arm,
    // this.finalPosition.getAngle().minus(ArmConstants.ZERO_ARM_OFFSET).getDegrees()));
    // } else {
    System.out.println("OUtside bumpers");
    addCommands(new ResetElevatorCommand(elevator),
        new RotateArmToAngleCommand(arm,
            this.finalPosition.getAngle().plus(ArmConstants.ZERO_ARM_OFFSET).getDegrees(),
            ArmConstants.SEQUENTIAL_ROTATION_ERROR_DEADBAND),
        // angle of the translation from the pivot to the target point is the angle the arm needs to rotate
        // however, the angle must be offset because the arm's 0-degree rotation is not actually vertical.
        new RunElevatorToPositionCommand(elevator,
            this.finalPosition.getNorm() - ElevatorConstants.LENGTH_FULLY_RETRACTED));
    // length of the translation from the pivot to the target point is the total length the elevator needs to span;
    // need to subtract the fully retracted length of the elevator to calculate how much it needs to extend.
    // }
    addRequirements(this.arm, this.elevator);
  }
}
