// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package overclocked.stl.localization;

/**
 * A Pose Estimator that combines the Vision Pose Estimate, the Odometry Pose Estimate, and the combined Vision-Odometry
 * Pose Estimate.
 */
public abstract class FinalPoseEstimator implements CombinedPoseEstimator {
  private VisionPoseEstimator visionPoseEstimate;
  private OdometryPoseEstimator_I odometryPoseEstimate;
  private VisionOdometryPoseEstimator visionOdometryPoseEstimator;

  FinalPoseEstimator(VisionPoseEstimator visionPoseEstimate, OdometryPoseEstimator_A odometryPoseEstimate,
      VisionOdometryPoseEstimator visionOdometryPoseEstimator) {
    this.visionPoseEstimate = visionPoseEstimate;
    this.odometryPoseEstimate = odometryPoseEstimate;
    this.visionOdometryPoseEstimator = visionOdometryPoseEstimator;
  }
}
