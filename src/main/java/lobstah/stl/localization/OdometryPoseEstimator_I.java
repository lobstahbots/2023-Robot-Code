// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.localization;

import edu.wpi.first.math.geometry.Pose2d;

/** An interface for estimating pose based on odometry. Can be calibrated with manual calibration. */
public interface OdometryPoseEstimator_I {

  public abstract Pose2d combinePoses();

  public abstract double calculateCombinedConfidence();

  public abstract ConfidencePose combinePosesWithConfidence();

  public abstract void calibrate(ManualCalibration calibration);

}
