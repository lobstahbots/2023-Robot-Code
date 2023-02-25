
package frc.robot.commands.scoring;

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
        // If starting position is inside bumper collision zone, first rotate to safety angle
        new ScoringSystemToPositionCommand(arm, elevator, () -> ScoringPosition.fromArmElevator(
            ScoringSystemConstants.BUMPER_AVOIDANCE_ANGLE, elevator.getExtension()),
            ScoringSystemConstants.BUMPER_AVOIDANCE_PRECISION)
                .unless(() -> !ScoringPosition
                    .fromArmElevator(arm.getRotation(), elevator.getExtension())
                    .isInsideBumperZone()),
        // Retract elevator unless close enough to final angle to not need to retract before rotating.
        new ScoringSystemToPositionCommand(arm, elevator, () -> ScoringPosition.fromArmElevator(
            arm.getRotation(), 0), ScoringSystemConstants.BUMPER_AVOIDANCE_PRECISION),
        // If target position is inside bumper collision zone, after retracting, rotate to safety angle and extend to
        // target extension sequentially.
        new SequentialCommandGroup(
            new ScoringSystemToPositionCommand(arm, elevator, () -> ScoringPosition.fromArmElevator(
                ScoringSystemConstants.BUMPER_AVOIDANCE_ANGLE, elevator.getExtension()),
                ScoringSystemConstants.BUMPER_AVOIDANCE_PRECISION),
            new ScoringSystemToPositionCommand(arm, elevator, () -> ScoringPosition.fromArmElevator(
                ScoringSystemConstants.BUMPER_AVOIDANCE_ANGLE, position.getElevatorExtension()),
                ScoringSystemConstants.BUMPER_AVOIDANCE_PRECISION)).unless(() -> !position.isInsideBumperZone()),
        // Finally, completely rotate and extend to target position.
        new ScoringSystemTowardsPositionCommand(arm, elevator, position));
  }
}
