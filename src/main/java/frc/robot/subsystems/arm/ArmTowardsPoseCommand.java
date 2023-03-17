// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmPose;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;

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
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.resetElevatorPID();
    arm.resetPivotPID();
  }

  @Override
  public void execute() {
    ArmPose currentPose = pose.get();
    double clampedExtension = MathUtil.clamp(currentPose.getExtension(), ElevatorConstants.MIN_EXTENSION_INCHES,
        ElevatorConstants.MAX_EXTENSION_INCHES);
    arm.setElevatorPIDGoal(clampedExtension);
    arm.feedElevatorPID();

    double clampedAngle =
        MathUtil.clamp(currentPose.getAngle().getDegrees(), PivotConstants.MIN_ROTATION_DEG,
            PivotConstants.MAX_ROTATION_DEG);
    arm.setPivotPIDGoal(clampedAngle);
    arm.feedPivotPID();
  }

  @Override
  public void end(boolean interrupted) {
    arm.setElevatorSpeed(0);
    arm.setPivotSpeed(0);
  }
}
