// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoringSystemConstants.ArmConstants;
import frc.robot.subsystems.Arm;

public class RotateArmToAngleCommand extends CommandBase {
  private final Arm arm;
  private DoubleSupplier angle;

  /**
   * Creates a command that rotates the {@link Arm} to a given angle in degrees.
   *
   * @param arm The {@link Arm} to control
   * @param angle A supplier for the angle in degrees to rotate the arm to
   */
  public RotateArmToAngleCommand(Arm arm, DoubleSupplier angleDegrees) {
    this.arm = arm;
    this.angle = angleDegrees;
    addRequirements(this.arm);
  }

  /**
   * Creates a command that rotates the {@link Arm} to a given angle in degrees.
   *
   * @param arm The {@link Arm} to control
   * @param angle The angle in degrees to rotate the arm to
   */
  public RotateArmToAngleCommand(Arm arm, double angleDegrees) {
    this(arm, () -> angleDegrees);
  }

  @Override
  public void initialize() {
    arm.resetPID();
  }

  @Override
  public void execute() {
    double clampedAngle =
        MathUtil.clamp(angle.getAsDouble(), ArmConstants.MIN_ROTATION_DEG, ArmConstants.MAX_ROTATION_DEG);
    arm.setPIDGoal(clampedAngle);
    arm.feedPID();
  }

  @Override
  public void end(boolean interrupted) {
    arm.setRotationSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}