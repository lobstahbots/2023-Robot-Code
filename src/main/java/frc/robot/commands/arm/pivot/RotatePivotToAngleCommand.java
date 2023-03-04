// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.subsystems.Arm;

public class RotatePivotToAngleCommand extends CommandBase {
  private final Arm.Pivot pivot;
  private DoubleSupplier angle;

  /**
   * Creates a command that rotates the {@link Arm.Pivot} to a given angle in degrees.
   *
   * @param arm The {@link Arm} to control
   * @param angle A supplier for the angle in degrees to rotate the arm to
   */
  public RotatePivotToAngleCommand(Arm arm, DoubleSupplier angleDegrees) {
    this.pivot = arm.getPivot();
    this.angle = angleDegrees;
    addRequirements(arm);
  }

  /**
   * Creates a command that rotates the {@link Arm.Pivot} to a given angle in degrees.
   *
   * @param arm The {@link Arm} to control
   * @param angle The angle in degrees to rotate the arm to
   */
  public RotatePivotToAngleCommand(Arm arm, double angleDegrees) {
    this(arm, () -> angleDegrees);
  }

  @Override
  public void initialize() {
    pivot.resetPID();
  }

  @Override
  public void execute() {
    double clampedAngle =
        MathUtil.clamp(angle.getAsDouble(), PivotConstants.MIN_ROTATION_DEG, PivotConstants.MAX_ROTATION_DEG);
    pivot.setPIDGoal(clampedAngle);
    pivot.feedPID();
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setRotationSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
