
package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ScoringPosition;
import frc.robot.Constants.ScoringSystemConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoringSystemTowardsPositionWithRetractionCommand extends SequentialCommandGroup {
  Arm arm;
  Elevator elevator;
  ScoringPosition position;

  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} towards a given position, retracting before
   * rotating if the difference in arm angle exceeds a certain threshold.
   * 
   * @param arm
   * @param elevator
   * @param position
   */
  public ScoringSystemTowardsPositionWithRetractionCommand(Arm arm, Elevator elevator, ScoringPosition position) {
    this.arm = arm;
    this.elevator = elevator;
    this.position = position;
    addRequirements(arm, elevator);

    addCommands(
        new ConditionalCommand( // If moving directly would cause collision with bumpers
            new SequentialCommandGroup( // Avoid bumpers
                new ScoringSystemToPositionCommand(arm, elevator,
                    () -> ScoringPosition.fromArmElevator(
                        ScoringSystemConstants.BUMPER_AVOIDANCE_ANGLE, elevator.getExtension()),
                    ScoringSystemConstants.BUMPER_AVOIDANCE_PRECISION),
                new ScoringSystemToPositionCommand(arm, elevator,
                    () -> ScoringPosition.fromArmElevator(ScoringSystemConstants.BUMPER_AVOIDANCE_ANGLE,
                        position.getElevatorExtension()),
                    ScoringSystemConstants.BUMPER_AVOIDANCE_PRECISION)),
            new SequentialCommandGroup( // Else if difference in arm angle exceeds threshold, retract and rotate first
                new ScoringSystemToPositionCommand(arm, elevator,
                    () -> ScoringPosition.fromArmElevator(Rotation2d.fromDegrees(arm.getAngle()), 0),
                    ScoringSystemConstants.RETRACT_BEFORE_ROTATING_PRECISION),
                new ScoringSystemToPositionCommand(arm, elevator,
                    ScoringPosition.fromArmElevator(position.getArmAngle(), 0),
                    ScoringSystemConstants.RETRACT_BEFORE_ROTATING_PRECISION))
                        .unless(() -> Math.abs(arm.getAngle() - position.getArmAngle()
                            .getDegrees()) < ScoringSystemConstants.RETRACT_BEFORE_ROTATING_ANGLE),
            this::willCollideWithBumpers),

        new ScoringSystemTowardsPositionCommand(arm, elevator, position));
  }

  private boolean willCollideWithBumpers() {
    ScoringPosition currentPosition =
        ScoringPosition.fromArmElevator(Rotation2d.fromDegrees(arm.getAngle()), elevator.getExtension());

    boolean currentlyUnderAngle =
        currentPosition.getArmAngle().getDegrees() < ScoringSystemConstants.BUMPER_AVOIDANCE_ANGLE.getDegrees();
    boolean targetUnderAngle =
        position.getArmAngle().getDegrees() < ScoringSystemConstants.BUMPER_AVOIDANCE_ANGLE.getDegrees();

    boolean currentlyInsideX = currentPosition.getX() < ScoringSystemConstants.BUMPER_AVOIDANCE_X;
    boolean targetInsideX = position.getX() < ScoringSystemConstants.BUMPER_AVOIDANCE_X;

    return currentlyUnderAngle && targetUnderAngle && (currentlyInsideX != targetInsideX);
  }
}
