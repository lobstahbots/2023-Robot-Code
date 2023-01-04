// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.localization;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

/** Add your docs here. */
public abstract class OdometryPoseEstimator_A implements OdometryPoseEstimator_I {
  public DifferentialDriveOdometry odometryController;

  public OdometryPoseEstimator_A(DifferentialDriveOdometry odometryController) {
    this.odometryController = odometryController;
  }

}
