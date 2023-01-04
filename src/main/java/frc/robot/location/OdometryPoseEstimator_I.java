// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;
import lobstah.stl.localization.ManualCalibration;

/** An interface for estimating pose based on odometry. Can be calibrated with manual calibration. */
public interface OdometryPoseEstimator_I {

  public abstract double calculateConfidence();

  public abstract Pose2d estimatePose();

  public abstract ConfidencePose calculatePoseWithConfidence();

  public abstract void calibrate(ManualCalibration calibration);

}
