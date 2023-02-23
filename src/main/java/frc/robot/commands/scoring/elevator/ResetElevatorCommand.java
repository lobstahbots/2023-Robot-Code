
package frc.robot.commands.scoring.elevator;

import frc.robot.subsystems.Elevator;

public class ResetElevatorCommand extends RetractElevatorCommand {
  /**
   * Creates a command that fully retracts the {@link Elevator} until the limit switch is flipped. If the limit switch
   * is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct for any
   * partial triggering.
   * 
   * @param elevator The {@link Elevator} to control
   */
  public ResetElevatorCommand(Elevator elevator) {
    super(elevator);
  }

  @Override
  public boolean isFinished() {
    return !this.needsToExtend && elevator.isRetracted();
  }

  @Override
  public void end(boolean interrupted) {
    elevator.resetEncoder();
    elevator.move(0);
  }
}
