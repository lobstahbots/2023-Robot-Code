
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ElevatorConstants;

public class ArmResetElevatorCommand extends CommandBase {
  private boolean needsToExtend;
  private final Arm arm;

  /**
   * Creates a command that fully retracts the {@link Arm.Elevator} until the limit switch is flipped. If the limit
   * switch is already flipped, the elevator extends to untrigger it and then retracts until it is flipped, to correct
   * for any partial triggering.
   * 
   * @param arm The {@link Arm} to control
   */
  public ArmResetElevatorCommand(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    this.needsToExtend = arm.isElevatorRetracted();
    SmartDashboard.putBoolean("Is Retracted Already", arm.isElevatorRetracted());
  }

  @Override
  public void execute() {
    if (this.needsToExtend) {
      if (arm.isElevatorRetracted()) {
        arm.setElevatorSpeed(ElevatorConstants.HOME_SPEED);
      } else {
        this.needsToExtend = false;
      }
    } else {
      if (!arm.isElevatorRetracted()) {
        arm.moveElevatorToLimitSwitch();
      } else {
        arm.setElevatorSpeed(0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return !this.needsToExtend && arm.isElevatorRetracted();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted == false) {
      arm.resetElevatorEncoder();
    }
    arm.setElevatorSpeed(0);
  }
}
