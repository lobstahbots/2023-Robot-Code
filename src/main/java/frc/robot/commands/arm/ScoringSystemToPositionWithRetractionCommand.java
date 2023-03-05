
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ArmPose;
import frc.robot.subsystems.Arm;

public class ScoringSystemToPositionWithRetractionCommand extends ParallelRaceGroup {
  private final Arm arm;
  private final ArmPose position;
  private final double threshold;

  /**
   * Creates a command that moves the {@link Arm} to a given position, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param position A supplier for the position to move to
   * @param threshold The threshold in inches for the system to be considered at the correct position
   */
  public ScoringSystemToPositionWithRetractionCommand(Arm arm, ArmPose position,
      double threshold) {
    this.arm = arm;
    this.position = position;
    this.threshold = threshold;

    this.addCommands(new ScoringSystemTowardsPositionWithRetractionCommand(arm, position),
        new WaitUntilCommand(this::isAtPosition));
  }

  private boolean isAtPosition() {
    ArmPose currentPosition =
        ArmPose.fromAngleExtension(arm.getPivot().getRotation(), arm.getElevator().getExtension());
    return position.getDistance(currentPosition) <= threshold;
  }
}
