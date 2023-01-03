// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstah.stl.localization;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class ManualCalibration {
  private CalibrationType type;
  private Supplier<Double> newValue;

  ManualCalibration(CalibrationType type, Supplier<Double> newValue) {
    this.type = type;
    this.newValue = newValue;
  }

  public CalibrationType getCalibrationType() {
    return type;
  }

  public double getNewValue() {
    return this.newValue.get();
  }
}
