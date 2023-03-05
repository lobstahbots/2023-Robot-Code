
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPose;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ScoringSystemTowardsPositionWithRetractionCommand extends SequentialCommandGroup {
  Arm arm;
  ArmPose position;

  /**
   * Creates a command that moves the {@link Arm} towards a given position, retracting before rotating if the difference
   * in arm angle exceeds a certain threshold.
   * 
   * @param arm
   * @param position
   */
  public ScoringSystemTowardsPositionWithRetractionCommand(Arm arm, ArmPose position) {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);

    addCommands(
        // If starting position is inside bumper collision zone, first rotate to safety angle
        new ScoringSystemToPositionCommand(
            arm,
            () -> ArmPose.fromArmElevator(ArmConstants.BUMPER_AVOIDANCE_ANGLE,
                arm.getElevator().getExtension()),
            ArmConstants.BUMPER_AVOIDANCE_PRECISION)
                .unless(() -> !ArmPose
                    .fromArmElevator(arm.getPivot().getRotation(), arm.getElevator().getExtension())
                    .isInsideBumperZone()),
        // Retract elevator unless close enough to final angle to not need to retract before rotating.
        new ScoringSystemToPositionCommand(
            arm, () -> ArmPose.fromArmElevator(
                arm.getPivot().getRotation(), 0),
            ArmConstants.BUMPER_AVOIDANCE_PRECISION),
        // If target position is inside bumper collision zone, after retracting, rotate to safety angle and extend to
        // target extension sequentially.
        new SequentialCommandGroup(
            new ScoringSystemToPositionCommand(arm, () -> ArmPose.fromArmElevator( // rotate
                ArmConstants.BUMPER_AVOIDANCE_ANGLE, arm.getElevator().getExtension()),
                ArmConstants.BUMPER_AVOIDANCE_PRECISION),
            new ScoringSystemToPositionCommand(arm, () -> ArmPose.fromArmElevator( //
                ArmConstants.BUMPER_AVOIDANCE_ANGLE, position.getElevatorExtension()),
                ArmConstants.BUMPER_AVOIDANCE_PRECISION)).unless(() -> !position.isInsideBumperZone()),
        // Finally, completely rotate and extend to target position.
        new ScoringSystemToPositionCommand(arm,
            () -> ArmPose.fromArmElevator(position.getArmAngle(), arm.getElevator().getExtension()),
            ArmConstants.BUMPER_AVOIDANCE_PRECISION),
        new ScoringSystemTowardsPositionCommand(arm, position));
  }
}
