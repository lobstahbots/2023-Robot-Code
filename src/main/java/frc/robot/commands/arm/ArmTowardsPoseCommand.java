// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmPose;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.subsystems.Arm;

public class ArmTowardsPoseCommand extends CommandBase {
  private final Arm arm;
  private final Supplier<ArmPose> pose;

  /**
   * Creates a command that moves the {@link Arm} towards a given pose.
   *
   * @param arm The {@link Arm} to control
   * @param pose The pose to move towards
   */
  public ArmTowardsPoseCommand(Arm arm, ArmPose pose) {
    this(arm, () -> pose);
  }

  /**
   * Creates a command that moves the {@link Arm} towards a given pose.
   *
   * @param arm The {@link Arm} to control
   * @param pose A supplier for the pose to move towards
   */
  public ArmTowardsPoseCommand(Arm arm, Supplier<ArmPose> pose) {
    this.arm = arm;
    this.pose = pose;
  }

  @Override
  public void initialize() {
    arm.getElevator().resetPID();
    arm.getPivot().resetPID();
  }

  @Override
  public void execute() {
    double clampedExtension = MathUtil.clamp(pose.get().getExtension(), ElevatorConstants.MIN_EXTENSION_INCHES,
        ElevatorConstants.MAX_EXTENSION_INCHES);
    arm.getElevator().setPIDGoal(clampedExtension);
    arm.getElevator().feedPID();

    double clampedAngle =
        MathUtil.clamp(pose.get().getAngle().getDegrees(), PivotConstants.MIN_ROTATION_DEG,
            PivotConstants.MAX_ROTATION_DEG);
    arm.getPivot().setPIDGoal(clampedAngle);
    arm.getPivot().feedPID();
  }

  @Override
  public void end(boolean interrupted) {
    arm.getElevator().move(0);
    arm.getPivot().setRotationSpeed(0);
  }
}
