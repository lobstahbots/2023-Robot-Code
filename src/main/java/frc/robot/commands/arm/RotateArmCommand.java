// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class RotateArmCommand extends CommandBase {

  public final Arm arm;
  public final DoubleSupplier speed;

  /**
   * Creates a command that rotates the {@link Arm} at the speed given by the supplier.
   *
   * @param arm The {@link Arm} to control
   * @param speed Supplier for the speed at which to rotate the arm
   */
  public RotateArmCommand(Arm arm, DoubleSupplier speed) {
    this.arm = arm;
    this.speed = speed;
    addRequirements(this.arm);
  }

  /**
   * Creates a command that rotates the {@link Arm} at a given speed.
   *
   * @param arm The {@link Arm} to control
   * @param speed The speed at which to rotate the arm
   */
  public RotateArmCommand(Arm arm, double speed) {
    this(arm, () -> speed);
  }

  @Override
  public void execute() {
    if ((arm.getAngle() > ArmConstants.kMaxRotationDeg && speed.getAsDouble() < 0)
        || (arm.getAngle() < ArmConstants.kMinRotationDeg && speed.getAsDouble() > 0)) {
      arm.setRotationSpeed(0.0);
      SmartDashboard.putBoolean("Arm Within Limit", false);
    } else {
      arm.setRotationSpeed(speed.getAsDouble());
      SmartDashboard.putBoolean("Arm Within Limit", true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.setRotationSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    // return arm.getMeasurement() >= ArmConstants.kMaxRotationRads + ArmConstants.kMaxRotationDeg
    // || arm.getMeasurement() <= ArmConstants.kMinRotationRads + kMinRotationDeg;
    return false;
  }
}
