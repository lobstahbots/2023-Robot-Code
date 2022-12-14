// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.localization;

/** A Pose Estimator that combines the Vision Pose Estimate and the Odometry Pose Estimate */
public abstract class VisionOdometryPoseEstimator implements CombinedPoseEstimator {
  private VisionPoseEstimator visionPoseEstimate;
  private OdometryPoseEstimator_I odometryPoseEstimate;

  VisionOdometryPoseEstimator(VisionPoseEstimator visionPoseEstimate, OdometryPoseEstimator_A odometryPoseEstimate) {
    this.visionPoseEstimate = visionPoseEstimate;
    this.odometryPoseEstimate = odometryPoseEstimate;
  }
}
