
package frc.robot.commands.scoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoringSystemToPositionCommand extends ParallelRaceGroup {
  private final Arm arm;
  private final Elevator elevator;
  private final Supplier<ScoringPosition> position;
  private final double threshold;

  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} to a given position, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param elevator The {@link Elevator} to control
   * @param position A supplier for the position to move to
   * @param threshold The threshold in inches for the system to be considered at the correct position
   */
  public ScoringSystemToPositionCommand(Arm arm, Elevator elevator, Supplier<ScoringPosition> position,
      double threshold) {
    this.arm = arm;
    this.elevator = elevator;
    this.position = position;
    this.threshold = threshold;

    this.addCommands(new ScoringSystemTowardsPositionCommand(arm, elevator, position),
        new WaitUntilCommand(this::isAtPosition));
  }

  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} to a given position, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param elevator The {@link Elevator} to control
   * @param position The position to move to
   * @param threshold The threshold in inches for the system to be considered at the correct position
   */
  public ScoringSystemToPositionCommand(Arm arm, Elevator elevator, ScoringPosition position, double threshold) {
    this(arm, elevator, () -> position, threshold);
  }

  private boolean isAtPosition() {
    ScoringPosition currentPosition =
        ScoringPosition.fromArmElevator(Rotation2d.fromDegrees(arm.getAngle()), elevator.getExtension());
    return position.get().getDistance(currentPosition) <= threshold;
  }
}
