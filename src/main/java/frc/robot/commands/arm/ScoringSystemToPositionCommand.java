
package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ArmPose;
import frc.robot.subsystems.Arm;

public class ScoringSystemToPositionCommand extends ParallelRaceGroup {
  private final Arm arm;
  private final Supplier<ArmPose> pose;
  private final double threshold;

  /**
   * Creates a command that moves the {@link Arm} to a given pose, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param pose A supplier for the pose to move to
   * @param threshold The threshold in inches for the system to be considered at the correct pose
   */
  public ScoringSystemToPositionCommand(Arm arm, Supplier<ArmPose> pose,
      double threshold) {
    this.arm = arm;
    this.pose = pose;
    this.threshold = threshold;

    this.addCommands(new ScoringSystemTowardsPositionCommand(arm, pose),
        new WaitUntilCommand(this::isAtPosition));
  }

  /**
   * Creates a command that moves the {@link Arm} to a given position, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param pose The pose to move to
   * @param threshold The threshold in inches for the arm to be considered at the correct pose
   */
  public ScoringSystemToPositionCommand(Arm arm, ArmPose pose, double threshold) {
    this(arm, () -> pose, threshold);
  }

  private boolean isAtPosition() {
    ArmPose currentPosition =
        ArmPose.fromAngleExtension(arm.getPivot().getRotation(), arm.getElevator().getExtension());
    return pose.get().getDistance(currentPosition) <= threshold;
  }
}
