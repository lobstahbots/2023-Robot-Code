// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;

/** A return type containing pose and confidence. */
public class ConfidencePose {
  public double confidence;
  public Pose2d pose;

  public ConfidencePose(double confidence, Pose2d pose) {
    this.confidence = confidence;
    this.pose = pose;
  }
}
