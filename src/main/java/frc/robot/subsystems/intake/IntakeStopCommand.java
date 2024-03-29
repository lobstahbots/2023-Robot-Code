// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Repeatedly sets the {@link Intake} speed to 0.
 */
public class IntakeStopCommand extends CommandBase {

  public final Intake intake;

  /**
   * Creates a command that sets the {@link Intake} speed to 0.
   *
   * @param intake The {@link Intake} to control
   */
  public IntakeStopCommand(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    intake.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
