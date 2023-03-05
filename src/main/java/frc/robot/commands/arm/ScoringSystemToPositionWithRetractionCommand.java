
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ArmPose;
import frc.robot.subsystems.Arm;

public class ScoringSystemToPositionWithRetractionCommand extends ParallelRaceGroup {
  private final Arm arm;
  private final ArmPose pose;
  private final double threshold;

  /**
   * Creates a command that moves the {@link Arm} to a given pose, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param pose A supplier for the pose to move to
   * @param threshold The threshold in inches for the arm to be considered at the correct pose
   */
  public ScoringSystemToPositionWithRetractionCommand(Arm arm, ArmPose pose,
      double threshold) {
    this.arm = arm;
    this.pose = pose;
    this.threshold = threshold;

    this.addCommands(new ScoringSystemTowardsPositionWithRetractionCommand(arm, pose),
        new WaitUntilCommand(this::isAtPose));
  }

  private boolean isAtPose() {
    ArmPose currentPosition =
        ArmPose.fromAngleExtension(arm.getPivot().getRotation(), arm.getElevator().getExtension());
    return pose.getDistance(currentPosition) <= threshold;
  }
}
