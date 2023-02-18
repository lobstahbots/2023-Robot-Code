// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MaintainArmAngleCommand extends CommandBase {
  private final Arm arm;

  public MaintainArmAngleCommand(Arm arm) {
    this.arm = arm;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {
    // System.out.println("Maintain Angle");
    double targetAngleDegrees = arm.getAngle();
    if (targetAngleDegrees > ArmConstants.kMaxRotationDeg) {
      targetAngleDegrees = ArmConstants.kMaxRotationDeg;
    }
    if (targetAngleDegrees < ArmConstants.kMinRotationDeg) {
      targetAngleDegrees = ArmConstants.kMinRotationDeg;
    }
    arm.setPIDGoal(targetAngleDegrees);
  }

  @Override
  public void execute() {
    arm.feedPID();
  }

  @Override
  public boolean isFinished() {
    return arm.getAngle() > ArmConstants.kMaxRotationDeg
        || arm.getAngle() < ArmConstants.kMinRotationDeg;
  }
}
