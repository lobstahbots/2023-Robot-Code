// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.localization;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public interface VisionPoseProvider {

  public abstract Pose2d getPoseEstimate();

  public abstract ConfidencePose getPoseEstimateWithConfidence();
}
