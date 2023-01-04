// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;
import lobstah.stl.localization.ConfidencePose;
import lobstah.stl.localization.VisionPoseProvider;

/** Add your docs here. */
public class VisionEstimator implements VisionPoseProvider {
  private double confidence;

  public VisionEstimator(double confidence) {
    this.confidence = 1;
  }

  public Pose2d getPoseEstimate() {
    // TODO: Use vision tracking to get pose estimate
    return null;
  }

  public ConfidencePose getPoseEstimateWithConfidence() {
    return new ConfidencePose(this.calculateConfidence(), this.getPoseEstimate());
  }

  public double calculateConfidence() {
    return this.confidence;
  }

}
