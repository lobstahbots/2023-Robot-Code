// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lobstahbots.stl.math;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class LobstahMath {

  /**
   * Scales a number on a range of values to a corresponding value on a different range
   * 
   * @param x The number to scale.
   * @param inputMin The original range's lower bound
   * @param inputMax The original range's upper bound
   * @param outputMin The new range's lower bound
   * @param outputMax The new range's upper bound
   */
  public static double scaleNumberToRange(double x, double inputMin, double inputMax, double outputMin,
      double outputMax) {
    if (inputMax < inputMin) {
      double temp = inputMin;
      inputMin = inputMax;
      inputMax = temp;
    }

    if (outputMax < outputMin) {
      double temp = outputMin;
      outputMin = outputMax;
      outputMax = temp;
    }
    double originalRange = inputMax - inputMin;
    double scaledRange = outputMax - outputMin;

    if (originalRange == 0) {
      System.out.println("Error: Cannot scale to a range of 0");
      return x;
    }

    x = MathUtil.clamp(x, inputMin, inputMax);

    double scaledValue = (((x - inputMin) * scaledRange) / originalRange) + outputMin;
    return scaledValue;
  }
}
