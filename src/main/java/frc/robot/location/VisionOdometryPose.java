// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;
import lobstah.stl.localization.ConfidencePose;
import lobstah.stl.localization.VisionOdometryPoseEstimator;
import lobstah.stl.localization.VisionPoseEstimator;

public class VisionOdometryPose implements VisionOdometryPoseEstimator {
  private final VisionPoseEstimator visionPoseEstimator;
  private final OdometryPoseEstimator odometryPoseEstimator;

  public VisionOdometryPose(VisionPoseEstimator visionPoseEstimator, OdometryPoseEstimator odometryPoseEstimator) {
    this.odometryPoseEstimator = odometryPoseEstimator;
    this.visionPoseEstimator = visionPoseEstimator;
    // TODO: Add DifferentialDrivePoseEstimator to combine odometry with vision tracking
  }

  public Pose2d combinePoses() {
    return odometryPoseEstimator.estimatePose();
  }

  public double calculateCombinedConfidence() {
    return (odometryPoseEstimator.calculateConfidence() + visionPoseEstimator.calculateCombinedConfidence()) / 2;
  }

  public ConfidencePose combinePosesWithConfidence() {
    return new ConfidencePose(this.calculateCombinedConfidence(), this.combinePoses());
  }

}
