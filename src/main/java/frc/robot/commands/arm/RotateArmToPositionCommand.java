// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class RotateArmToPositionCommand extends CommandBase {
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
    this.finalPosition = finalPosition;
  }

  @Override
  public void initialize() {
    this.arm.enable();
  }

  @Override
  public void execute() {
    Translation2d currentPose = new Translation2d(elevator.getExtension(), new Rotation2d(arm.getMeasurement()));
    this.arm.setGoal(this.finalPosition.getAngle().getRadians());
    this.elevator.setGoal(this.finalPosition.getNorm());
    SmartDashboard.putNumber("Intake X Component", currentPose.getX());
    SmartDashboard.putNumber("Intake Y Component", currentPose.getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
