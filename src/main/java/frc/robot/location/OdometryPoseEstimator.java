// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.location;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import lobstah.stl.localization.ConfidencePose;
import lobstah.stl.localization.ManualCalibration;
import lobstah.stl.localization.OdometryPoseEstimator_A;

public class OdometryPoseEstimator extends OdometryPoseEstimator_A {
  private double confidence;

  public OdometryPoseEstimator(DifferentialDriveOdometry odometry) {
    super(odometry);
    this.confidence = 1;
  }

  public double calculateConfidence() {
    return this.confidence;
  }

  public Pose2d estimatePose() {
    return this.odometryController.getPoseMeters();
  }

  public ConfidencePose calculatePoseWithConfidence() {
    return new ConfidencePose(this.calculateConfidence(), this.estimatePose());
  }

  public void calibrate(ManualCalibration calibration) {
    // TODO: Implement manual calibration
    switch (calibration.getCalibrationType()) {
      case X_POSITION:

      case Y_POSITION:

      case GYRO_ANGLE:

      default:
    }

  }


}
