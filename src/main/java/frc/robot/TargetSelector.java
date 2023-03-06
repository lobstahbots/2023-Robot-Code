// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import lobstah.stl.math.LobstahMath;

/**
 * Selects a target column and row in teleop. Can select column and row in two modes: incrementally changing row and
 * column, or Maxwell mode (individually selecting grid, column, and row).
 */
public class TargetSelector implements Sendable {
  private int targetColumn = 0;
  private int targetRow = 0;
  private int selectedMaxwellTargetGrid = -1;
  private int selectedMaxwellTargetColumn = -1;
  private int selectedMaxwellTargetRow = -1;
  private boolean inMaxwellMode = false;
  private boolean columnSelected = false;
  private boolean gridSelected = false;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Teleop Target Selector");
    builder.addBooleanProperty("In Maxwell Mode", this::getMode, null);
    builder.addIntegerProperty("Row Selected", this::getRow, null);
    builder.addIntegerProperty("Column Selected", this::getColumn, null);
    builder.addIntegerProperty("Maxwell Grid", this::getMaxwellGrid, null);
    builder.addIntegerProperty("Maxwell Column", this::getMaxwellColumn, null);
    builder.addIntegerProperty("Maxwell Row", this::getMaxwellRow, null);
  }

  /**
   * @return Whether or not the selector is in Maxwell mode.
   */
  public boolean getMode() {
    return inMaxwellMode;
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
   * @return The Maxwell selected grid.
   */
  public int getMaxwellGrid() {
    return selectedMaxwellTargetGrid;
  }

  /**
   * @return The Maxwell selected column.
   */
  public int getMaxwellColumn() {
    return selectedMaxwellTargetColumn;
  }

  /**
   * @return the Maxwell selected row.
   */
  public int getMaxwellRow() {
    return selectedMaxwellTargetRow;
  }


  /**
   * Sets the mode to the given mode.
   */
  public void setMode(boolean mode) {
    inMaxwellMode = mode;
  }

  /**
   * Resets Maxwell selected vales and sets Maxwell mode.
   * 
   * @param newMode The mode to change to.
   */
  public void resetSelection(boolean newMode) {
    setMode(newMode);
    columnSelected = false;
    gridSelected = false;
    selectedMaxwellTargetGrid = -1;
    selectedMaxwellTargetColumn = -1;
    selectedMaxwellTargetRow = -1;
  }

  /**
   * Changes the official selected column, wrapping it between 0 and 8.
   * 
   * @param change The amount to change the selected column by.
   */
  public void changeColumn(int change) {
    targetColumn += change;
    targetColumn = LobstahMath.inputModulus(targetColumn, -1, 8);
  }

  /**
   * Changes the official selected column, wrapping it between 0 and 2.
   * 
   * @param change The amount to change the selected row by.
   */
  public void changeRow(int change) {
    targetRow += change;
    targetRow = LobstahMath.inputModulus(targetRow, -1, 2);
  }

  /**
   * Sets Maxwell grid, column, or row to the provided index depending on which values are already selected. Only sets
   * official selected column and row once Maxwell grid, column, and row have all been selected.
   * 
   * @param index The value to set the Maxwell grid, column, or row to.
   */
  public void setTargetInMaxwellMode(int index) {
    if (gridSelected && columnSelected) {
      selectedMaxwellTargetRow = index;
      targetRow = selectedMaxwellTargetRow;
      targetColumn = 3 * selectedMaxwellTargetGrid + selectedMaxwellTargetColumn;
    } else if (gridSelected) {
      selectedMaxwellTargetColumn = index;
      columnSelected = true;
    } else {
      selectedMaxwellTargetGrid = index;
      gridSelected = true;
    }
  }
}
