// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SpinIntakeCommand extends CommandBase {

  public final Intake intake;
  public final double speed;

  /**
   * Creates a command that spins the {@link Intake} at a given speed.
   *
   * @param intake The {@link Intake} to control
   * @param speed The speed at which to spin the intake
   */
  public SpinIntakeCommand(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    intake.setSpinSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setSpinSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
