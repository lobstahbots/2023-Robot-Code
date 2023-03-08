
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPose;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmTowardsPoseWithRetractionCommand extends SequentialCommandGroup {
  Arm arm;
  ArmPose pose;

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
    addRequirements(arm);

    addCommands(
        // If starting position is inside bumper collision zone, first rotate to safety angle
        new ArmToPoseCommand(
            arm,
            () -> ArmPose.fromAngleExtension(ArmConstants.BUMPER_AVOIDANCE_ANGLE,
                arm.getElevator().getExtension()),
            ArmConstants.BUMPER_AVOIDANCE_ANGLE_PRECISION, ArmConstants.BUMPER_AVOIDANCE_EXTENSION_PRECISION)
                .unless(() -> !arm.getPose()
                    .isInsideBumperZone()),
        // Retract elevator unless close enough to final angle to not need to retract before rotating.
        new ArmToPoseCommand(
            arm, () -> ArmPose.fromAngleExtension(
                arm.getPivot().getRotation(), 0),
            ArmConstants.BUMPER_AVOIDANCE_ANGLE_PRECISION, ArmConstants.BUMPER_AVOIDANCE_EXTENSION_PRECISION),
        // If target position is inside bumper collision zone, after retracting, rotate to safety angle and extend to
        // target extension sequentially.
        new SequentialCommandGroup(
            new ArmToPoseCommand(arm, () -> ArmPose.fromAngleExtension( // rotate
                ArmConstants.BUMPER_AVOIDANCE_ANGLE, arm.getElevator().getExtension()),
                ArmConstants.BUMPER_AVOIDANCE_ANGLE_PRECISION, ArmConstants.BUMPER_AVOIDANCE_EXTENSION_PRECISION),
            new ArmToPoseCommand(arm, () -> ArmPose.fromAngleExtension( //
                ArmConstants.BUMPER_AVOIDANCE_ANGLE, pose.getExtension()),
                ArmConstants.BUMPER_AVOIDANCE_ANGLE_PRECISION, ArmConstants.BUMPER_AVOIDANCE_EXTENSION_PRECISION))
                    .unless(() -> !pose.isInsideBumperZone()),
        // Finally, completely rotate and extend to target position.
        new ArmToPoseCommand(arm,
            () -> ArmPose.fromAngleExtension(pose.getAngle(), arm.getElevator().getExtension()),
            ArmConstants.SWITCH_TO_PARALLEL_ANGLE_PRECISION, ArmConstants.BUMPER_AVOIDANCE_EXTENSION_PRECISION),
        new ArmTowardsPoseCommand(arm, pose));
  }
}
