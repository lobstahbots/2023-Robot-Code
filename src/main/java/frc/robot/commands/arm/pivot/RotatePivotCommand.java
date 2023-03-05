// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.pivot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.subsystems.Arm;

public class RotatePivotCommand extends CommandBase {

  public final Arm.Pivot pivot;
  public final DoubleSupplier speed;

  /**
   * Creates a command that rotates the {@link Arm.Pivot} at the speed given by the supplier.
   *
   * @param arm The {@link Arm} to control
   * @param speed Supplier for the speed at which to rotate the pivot
   */
  public RotatePivotCommand(Arm arm, DoubleSupplier speed) {
    this.pivot = arm.getPivot();
    this.speed = speed;
    addRequirements(arm);
  }

  /**
   * Creates a command that rotates the {@link Arm.Pivot} at a given speed.
   *
   * @param arm The {@link Arm} to control
   * @param speed The speed at which to rotate the pivot
   */
  public RotatePivotCommand(Arm arm, double speed) {
    this(arm, () -> speed);
  }

  @Override
  public void execute() {
    if ((pivot.getAngle() > PivotConstants.MAX_ROTATION_DEG && speed.getAsDouble() > 0)
        || (pivot.getAngle() < PivotConstants.MIN_ROTATION_DEG && speed.getAsDouble() < 0)) {
      pivot.setRotationSpeed(0.0);
    } else {
      pivot.setRotationSpeed(speed.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setRotationSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
