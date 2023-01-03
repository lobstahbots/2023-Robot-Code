// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package overclocked.stl.localization;

import edu.wpi.first.math.geometry.Pose2d;

/** A Pose Estimator that combines the poses and confidences of multiple Pose Estimates. */
public interface CombinedPoseEstimator {

  public abstract Pose2d combinePoses();

  public abstract double calculateCombinedConfidence();

  public abstract ConfidencePose combinePosesWithConfidence();
}
