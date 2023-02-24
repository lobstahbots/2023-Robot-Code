
package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ScoringPosition;
import frc.robot.Constants.ScoringPositionConstants;
import frc.robot.Constants.ScoringSystemConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoringSystemTowardsPositionWithRetractionCommand extends SequentialCommandGroup {
  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} towards a given position, retracting before
   * rotating if the difference in arm angle exceeds a certain threshold.
   * 
   * @param arm
   * @param elevator
   * @param position
   */
  public ScoringSystemTowardsPositionWithRetractionCommand(Arm arm, Elevator elevator, ScoringPosition position) {
    addRequirements(arm, elevator);

    addCommands(new SequentialCommandGroup(new ScoringSystemToPositionCommand(arm, elevator,
        () -> ScoringPosition.fromArmElevator(
            ScoringPositionConstants.OUTSIDE_BUMPERS.getArmAngle(), elevator.getExtension()),
        ScoringSystemConstants.AVOID_BUMPERS_PRECISION),
        new ScoringSystemToPositionCommand(arm, elevator,
            () -> ScoringPosition.fromArmElevator(ScoringPositionConstants.OUTSIDE_BUMPERS.getArmAngle(),
                ScoringPositionConstants.OUTSIDE_BUMPERS.getElevatorExtension()),
            ScoringSystemConstants.AVOID_BUMPERS_PRECISION))
                .unless(() -> !ScoringPosition
                    .fromArmElevator(Rotation2d.fromDegrees(arm.getAngle()), elevator.getExtension())
                    .isInsideXYZone(ScoringPositionConstants.BUMPER_ZONE_X, ScoringPositionConstants.BUMPER_ZONE_Y)
                    && !position.isInsideXYZone(ScoringPositionConstants.BUMPER_ZONE_X,
                        ScoringPositionConstants.BUMPER_ZONE_Y)),
        new SequentialCommandGroup(
            new ScoringSystemToPositionCommand(arm, elevator,
                () -> ScoringPosition.fromArmElevator(Rotation2d.fromDegrees(arm.getAngle()), 0),
                ScoringSystemConstants.RETRACT_BEFORE_ROTATING_PRECISION),
            new ScoringSystemToPositionCommand(arm, elevator,
                ScoringPosition.fromArmElevator(position.getArmAngle(), 0),
                ScoringSystemConstants.RETRACT_BEFORE_ROTATING_PRECISION))
                    .unless(() -> Math.abs(arm.getAngle()
                        - position.getArmAngle().getDegrees()) < ScoringSystemConstants.RETRACT_BEFORE_ROTATING_ANGLE),
        new ScoringSystemTowardsPositionCommand(arm, elevator, position));
  }
}
