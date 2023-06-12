// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConstructLaterCommand extends CommandBase {
  private final Supplier<Command> commandSupplier;
  private Command command;

  public ConstructLaterCommand(Supplier<Command> commandSupplier) {
    this.commandSupplier = commandSupplier;
    m_requirements.addAll(commandSupplier.get().getRequirements());
  }

  @Override
  public void initialize() {
    command = commandSupplier.get();
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
