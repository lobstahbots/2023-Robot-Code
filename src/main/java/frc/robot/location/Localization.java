// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;
import lobstah.stl.localization.CombinedPoseEstimator;
import lobstah.stl.localization.ConfidencePose;

public class Localization implements CombinedPoseEstimator {
  private final OdometryPoseEstimator odometryPoseEstimator;
  private final VisionEstimator visionEstimator;
  private final VisionOdometryPose visionOdometryPose;

  public Localization(OdometryPoseEstimator odometryPoseEstimator, VisionEstimator visionEstimator,
      VisionOdometryPose visionOdometryPose) {
    this.odometryPoseEstimator = odometryPoseEstimator;
    this.visionEstimator = visionEstimator;
    this.visionOdometryPose = visionOdometryPose;
  }

  public Pose2d combinePoses() {
    return visionOdometryPose.combinePoses();
  }

  public double calculateCombinedConfidence() {
    return visionOdometryPose.calculateCombinedConfidence();
  }

  public ConfidencePose combinePosesWithConfidence() {
    return new ConfidencePose(this.calculateCombinedConfidence(), this.combinePoses());
  }
}
