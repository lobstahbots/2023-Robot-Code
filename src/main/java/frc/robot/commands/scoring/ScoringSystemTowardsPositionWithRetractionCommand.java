
package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ScoringPosition;
import frc.robot.Constants.ScoringSystemConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ScoringSystemTowardsPositionWithRetractionCommand extends SequentialCommandGroup {
  public ScoringSystemTowardsPositionWithRetractionCommand(Arm arm, Elevator elevator, ScoringPosition position) {
    addRequirements(arm, elevator);

    addCommands(
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
