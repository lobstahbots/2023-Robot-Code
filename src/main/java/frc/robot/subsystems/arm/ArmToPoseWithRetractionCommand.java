
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ArmPose;
import lobstah.stl.command.ConstructLaterCommand;

public class ArmToPoseWithRetractionCommand extends ParallelRaceGroup {
  private final Arm arm;
  private final ArmPose pose;
  private double angleThreshold = Double.MAX_VALUE;
  private double extensionThreshold = Double.MAX_VALUE;
  private double cartesianThreshold = Double.MAX_VALUE;

  /**
   * Creates a command that moves the {@link Arm} to a given pose, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param pose The pose to move to
   * @param cartesianThreshold The Cartesian threshold (distance in inches) for the system to be considered at the
   *          correct pose
   */
  public ArmToPoseWithRetractionCommand(Arm arm, ArmPose pose,
      double cartesianThreshold) {
    this.arm = arm;
    this.pose = pose;
    this.cartesianThreshold = cartesianThreshold;

    this.addCommands(new ConstructLaterCommand(() -> new ArmTowardsPoseWithRetractionCommand(arm, pose)),
        new WaitUntilCommand(this::isAtPose));
  }

  /**
   * Creates a command that moves the {@link Arm} to a given pose, then finishes.
   *
   * @param arm The {@link Arm} to control
   * @param pose The pose to move to
   * @param angleThreshold The threshold in degrees for the pivot angle for the system to be considered at the correct
   *          pose
   * @param extensionThreshold The threshold in inches for the elevator extension for the system to be considered at the
   *          correct pose
   */
  public ArmToPoseWithRetractionCommand(Arm arm, ArmPose pose,
      double angleThreshold, double extensionThreshold) {
    this.arm = arm;
    this.pose = pose;
    this.angleThreshold = angleThreshold;
    this.extensionThreshold = extensionThreshold;

    this.addCommands(new ConstructLaterCommand(() -> new ArmTowardsPoseWithRetractionCommand(arm, pose)),
        new WaitUntilCommand(this::isAtPose));
  }

  private boolean isAtPose() {
    return pose.getDistance(arm.getPose()) <= cartesianThreshold
        && Math.abs(pose.getExtension() - arm.getPose().getExtension()) <= extensionThreshold
        && Math.abs(pose.getAngle().minus(arm.getPose().getAngle()).getDegrees()) <= angleThreshold;
  }
}
