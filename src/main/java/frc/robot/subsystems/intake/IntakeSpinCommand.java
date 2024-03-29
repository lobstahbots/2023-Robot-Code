// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeSpinCommand extends CommandBase {

  public final Intake intake;
  public final double voltage;

  /**
   * Creates a command that spins the {@link Intake} at a given speed.
   *
   * @param intake The {@link Intake} to control
   * @param voltage The speed at which to spin the intake
   */
  public IntakeSpinCommand(Intake intake, double voltage) {
    this.intake = intake;
    this.voltage = voltage;
    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    intake.setSpinVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setSpinVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
