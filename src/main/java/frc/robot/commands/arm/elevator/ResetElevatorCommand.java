
package frc.robot.commands.arm.elevator;

import frc.robot.subsystems.Arm;

public class ResetElevatorCommand extends RetractElevatorCommand {
  /**
   * Creates a command that fully retracts the {@link Arm.Elevator} until the limit switch is flipped. If the limit switch
   * is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct for any
   * partial triggering.
   * 
   * @param arm The {@link Arm} to control
   */
  public ResetElevatorCommand(Arm arm) {
    super(arm);
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
