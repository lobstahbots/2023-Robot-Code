
package frc.robot.commands.arm.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.arm.Elevator;

public class ResetElevatorCommand extends CommandBase {
  private boolean needsToExtend;
  private final Elevator elevator;

  /**
   * Creates a command that fully retracts the {@link Arm.Elevator} until the limit switch is flipped. If the limit
   * switch is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct
   * for any partial triggering.
   * 
   * @param arm The {@link Arm} to control
   */
  public ResetElevatorCommand(Arm arm) {
    this.elevator = arm.getElevator();
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    this.needsToExtend = elevator.isRetracted();
    SmartDashboard.putBoolean("Is Retracted Already", elevator.isRetracted());
  }

  @Override
  public void execute() {
    if (this.needsToExtend) {
      if (elevator.isRetracted()) {
        elevator.move(ElevatorConstants.HOME_SPEED);
      } else {
        this.needsToExtend = false;
      }
    } else {
      if (!elevator.isRetracted()) {
        elevator.moveToLimitSwitch();
      } else {
        elevator.move(0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return !this.needsToExtend && elevator.isRetracted();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted == false) {
      elevator.resetEncoder();
    }
    elevator.move(0);
  }
}
