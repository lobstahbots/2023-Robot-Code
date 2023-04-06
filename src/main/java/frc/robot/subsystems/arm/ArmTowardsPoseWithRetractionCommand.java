
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPose;
import frc.robot.Constants.ArmConstants;

public class ArmTowardsPoseWithRetractionCommand extends SequentialCommandGroup {
  Arm arm;
  ArmPose pose;
  double anglePrecision = ArmConstants.EXTENDING_PARALLEL_ANGLE_PRECISION;
  double extendingPrecision = ArmConstants.EXTENDING_R_PRECISION;

  /**
   * Creates a command that moves the {@link Arm} towards a given pose, retracting before rotating if the difference in
   * pivot angle exceeds a certain threshold.
   * 
   * @param arm The {@link Arm} to control
   * @param pose The pose to move towards
   */
  public ArmTowardsPoseWithRetractionCommand(Arm arm, ArmPose pose) {
    this.arm = arm;
    this.pose = pose;

    boolean needsToAvoidBumpers = pose.getY() < ArmConstants.BUMPER_AVOIDANCE_HEIGHT
        && arm.getPose().getY() < ArmConstants.BUMPER_AVOIDANCE_HEIGHT;

    if (needsToAvoidBumpers) {
      anglePrecision = 5;
      extendingPrecision = 1;
    }

    addRequirements(arm);

    addCommands(
        // If starting position is inside bumper collision zone, first rotate to safety angle
        new ArmToPoseCommand(
            arm,
            () -> ArmPose.fromAngleExtension(ArmConstants.BUMPER_AVOIDANCE_ANGLE,
                arm.getExtension()),
            ArmConstants.BUMPER_AVOIDANCE_THETA_PRECISION, ArmConstants.BUMPER_AVOIDANCE_R_PRECISION)
                .unless(() -> !arm.getPose()
                    .isInsideBumperZone()),
        // Retract elevator unless close enough to final angle to not need to retract before rotating.
        new ArmToPoseCommand(
            arm, () -> ArmPose.fromAngleExtension(
                arm.getRotation(), 0),
            anglePrecision, extendingPrecision, false),
        // If target position is inside bumper collision zone, after retracting, rotate to safety angle and extend to
        // target extension sequentially.
        // new SequentialCommandGroup(
        // new ArmToPoseCommand(arm, () -> ArmPose.fromAngleExtension( // rotate
        // ArmConstants.BUMPER_AVOIDANCE_ANGLE, arm.getExtension()),
        // anglePrecision, extensionPrecision, false),
        // new ArmToPoseCommand(arm, () -> ArmPose.fromAngleExtension( //
        // ArmConstants.BUMPER_AVOIDANCE_ANGLE, pose.getExtension()),
        // anglePrecision, extensionPrecision, false))
        // .unless(() -> !pose.isInsideBumperZone()),
        // Finally, completely rotate and extend to target position.
        new ArmToPoseCommand(arm,
            () -> ArmPose.fromAngleExtension(pose.getAngle(), 0),
            anglePrecision, ArmConstants.BUMPER_AVOIDANCE_R_PRECISION, false),
        new ArmTowardsPoseCommand(arm, pose, false));
  }
}
