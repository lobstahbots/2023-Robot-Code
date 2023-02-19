
package frc.robot.commands.arm.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ResetElevatorCommand extends CommandBase {
  private boolean needsToExtend;
  private final Elevator elevator;

  /**
   * Creates a command that fully retracts the {@link Elevator} until the limit switch is flipped. If the limit switch
   * is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct for any
   * partial triggering.
   * 
   * @param elevator The {@link Elevator} to control
   */
  public ResetElevatorCommand(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(this.elevator);
  }

  @Override
  public void initialize() {
    this.needsToExtend = elevator.isRetracted();
  }

  @Override
  public void execute() {

    if (elevator.isRetracted()) {
      if (this.needsToExtend) {
        elevator.move(ElevatorConstants.HOME_SPEED);
      }
    } else {
      this.needsToExtend = false;
      elevator.moveToLimitSwitch();
    }
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
