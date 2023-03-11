
package frc.robot.auton;

import java.util.HashMap;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class AutonChooser implements Sendable {
  private final String[] routineOptions = {
      "None", "Line", "Score", "Score + Line"
  };
  private String routine = "None";

  private final HashMap<String, Integer> startingColumnOptions = new HashMap<>();
  private int startingColumn = 0;

  private final HashMap<String, Integer> crossingSideOptions = new HashMap<>();
  private int crossingSide = 0;

  private final HashMap<String, Integer> rowOptions = new HashMap<>();
  private int row = 2;

  public AutonChooser() {
    startingColumnOptions.put("1", 0);
    startingColumnOptions.put("2", 1);
    startingColumnOptions.put("3", 2);
    startingColumnOptions.put("4", 3);
    startingColumnOptions.put("5", 4);
    startingColumnOptions.put("6", 5);
    startingColumnOptions.put("7", 6);
    startingColumnOptions.put("8", 7);
    startingColumnOptions.put("9", 8);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringArrayProperty("routine/options", () -> this.routineOptions, null);
    builder.addStringProperty("routine", null, (routine) -> {
      this.routine = routine;
      updateOptions();
    });

    builder.addStringProperty("starting column", null, (col) -> {
      this.startingColumn = startingColumnOptions.get(col);
      updateOptions();
    });
    builder.addStringArrayProperty("starting column/options",
        () -> crossingSideOptions.keySet().toArray(String[]::new), null);

    builder.addStringProperty("crossing side", null, (side) -> this.crossingSide = crossingSideOptions.get(side));
    builder.addStringArrayProperty("crossing side/options",
        () -> crossingSideOptions.keySet().toArray(String[]::new), null);

    builder.addStringProperty("row", null, (row) -> this.row = rowOptions.get(row));
    builder.addStringArrayProperty("row/options",
        () -> rowOptions.keySet().toArray(String[]::new), null);
  }

  private void updateOptions() {
    crossingSideOptions.clear();
    if (startingColumn >= 4 && startingColumn <= 6
        && (routine == "Line" || routine == "Score + Line")) {
      crossingSideOptions.clear();
      crossingSideOptions.put("Left", 0);
      crossingSideOptions.put("Right", 1);
    }

    rowOptions.clear();
    if (routine == "Score" || routine == "Score + Line") {
      rowOptions.put("Low", 0);
      rowOptions.put("Mid", 1);
      rowOptions.put("High", 2);
    }
  }

  public String getRoutine() {
    return routine;
  }

  public int getStartingColumn() {
    return startingColumn;
  }

  public int getCrossingSide() {
    return crossingSide;
  }

  public int getRow() {
    return row;
  }
}
