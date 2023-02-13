// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class RotateArmToAngleCommand extends CommandBase {
  private final Arm arm;
  private double targetAngleDegrees;

  /**
   * Creates a command that rotates the {@link Arm} to a given angle in degrees.
   *
   * @param arm The {@link Arm} to control
   * @param angle The angle in degrees to rotate the arm to
   */
  public RotateArmToAngleCommand(Arm arm, double angleDegrees) {
    this.arm = arm;
    this.targetAngleDegrees = angleDegrees;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    if (this.targetAngleDegrees > ArmConstants.kMaxRotationDeg) {
      targetAngleDegrees = ArmConstants.kMaxRotationDeg;
    }
    if (this.targetAngleDegrees < ArmConstants.kMinRotationDeg) {
      targetAngleDegrees = ArmConstants.kMinRotationDeg;
    }
  }

  @Override
  public void execute() {
    this.arm.setPIDGoal(targetAngleDegrees);
  }

  @Override
  public boolean isFinished() {
    return arm.getAngle() > ArmConstants.kMaxRotationDeg
        || arm.getAngle() < ArmConstants.kMinRotationDeg
        || Math.abs(this.targetAngleDegrees - arm.getAngle()) < ArmConstants.ROTATION_ERROR_DEADBAND;
  }

}
