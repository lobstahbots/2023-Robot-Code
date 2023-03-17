
package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose2d;

/** An estimated pose based on pipeline result that includes pose ambiguity */
public class EstimatedRobotPose {
  /** The estimated pose */
  public final Pose2d estimatedPose;

  /** The estimated time the frame used to derive the robot pose was taken */
  public final double timestampSeconds;

  /** The pose confidence */
  public final double confidence;

  /** The area of the tracked target */
  public final double targetArea;

  /**
   * Constructs an EstimatedRobotPose
   *
   * @param estimatedPose estimated pose
   * @param timestampSeconds timestamp of the estimate
   */
  public EstimatedRobotPose(Pose2d estimatedPose, double timestampSeconds, double confidence, double targetArea) {
    this.estimatedPose = estimatedPose;
    this.timestampSeconds = timestampSeconds;
    this.confidence = confidence;
    this.targetArea = targetArea;
  }
}
