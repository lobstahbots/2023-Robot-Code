
package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ScoringPosition;
import frc.robot.subsystems.Arm;

public class ScoringSystemToPositionCommand extends ParallelRaceGroup {
  private final Arm arm;
  private final Supplier<ScoringPosition> position;
  private final double threshold;

  /**
   * Creates a command that moves the {@link Arm} to a given position, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param position A supplier for the position to move to
   * @param threshold The threshold in inches for the system to be considered at the correct position
   */
  public ScoringSystemToPositionCommand(Arm arm, Supplier<ScoringPosition> position,
      double threshold) {
    this.arm = arm;
    this.position = position;
    this.threshold = threshold;

    this.addCommands(new ScoringSystemTowardsPositionCommand(arm, position),
        new WaitUntilCommand(this::isAtPosition));
  }

  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} to a given position, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param position The position to move to
   * @param threshold The threshold in inches for the system to be considered at the correct position
   */
  public ScoringSystemToPositionCommand(Arm arm, ScoringPosition position, double threshold) {
    this(arm, () -> position, threshold);
  }

  private boolean isAtPosition() {
    ScoringPosition currentPosition =
        ScoringPosition.fromArmElevator(arm.getPivot().getRotation(), arm.getElevator().getExtension());
    return position.get().getDistance(currentPosition) <= threshold;
  }
}
