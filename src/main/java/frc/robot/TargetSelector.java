// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Selects a target column and row in teleop. Can select column and row in two modes: incrementally changing row and
 * column, or Maxwell mode (individually selecting grid, column, and row).
 */
public class TargetSelector implements Sendable {
  private int targetColumn = 0;
  private int targetRow = 0;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Teleop Target Selector");
    builder.addIntegerProperty("Row Selected", this::getRow, null);
    builder.addIntegerProperty("Column Selected", this::getColumn, null);
  }


  /**
   * @return The official selected column.
   */
  public int getColumn() {
    return targetColumn;
  }

  /**
   * @return The official selected row.
   */
  public int getRow() {
    return targetRow;
  }

  /**
   * Changes the official selected column, wrapping it between 0 and 8.
   * 
   * @param change The amount to change the selected column by.
   */
  public void changeColumn(int change) {
    targetColumn += change;
    targetColumn = Math.floorMod(targetColumn, 9);
  }

  /**
   * Changes the official selected column, wrapping it between 0 and 2.
   * 
   * @param change The amount to change the selected row by.
   */
  public void changeRow(int change) {
    targetRow += change;
    targetRow = Math.floorMod(targetRow, 3);
  }
}
