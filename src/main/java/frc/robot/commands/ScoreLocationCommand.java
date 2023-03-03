package frc.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreLocationCommand extends CommandBase {

  private final boolean[][][] gridValues = new boolean[3][3][3];
  private final SuppliedValueWidget<Boolean>[][][] gridWidgets = new SuppliedValueWidget[3][3][3];
  private final BooleanSupplier hasCone;
  private final BooleanSupplier hasCube;

  private int selectedRow = 1;
  private int selectedColumn = 1;
  
  public ScoreLocationCommand(BooleanSupplier hasCone, BooleanSupplier hasCube) {
    this.hasCone = hasCone;
    this.hasCube = hasCube;

    var gridsTab = Shuffleboard.getTab("Grids");
    var gridsLayout = gridsTab.getLayout("Grids", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 9, "Number of rows", 3, "Label Position", "HIDDEN"))
        .withSize(9, 3)
        .withPosition(0, 0);

    for(int gridIndex = 0; gridIndex < 3; gridIndex++) {
      for(int columnIndex = 0; columnIndex < 3; columnIndex++) {
        if (1 == columnIndex) {
          // cubes
          for (int rowId = 0; rowId < 3; rowId++) {
            makeBox(gridsLayout, "Cube", gridIndex, columnIndex, rowId, Color.kPurple);
          }
        } else {
          // cones
          for (int rowId = 0; rowId < 3; rowId++) {
            makeBox(gridsLayout, "Cone", gridIndex, columnIndex, rowId, Color.kYellow);
          }
        }
      }
    }
    refreshGrid();
  }

  @Override
  public void execute() {
    // check for mismatch
    boolean match = true;
    if (selectedRow == 0) {
      match = true;
    } else if(hasCone.getAsBoolean()) {
      match = selectedColumn == 0 || selectedColumn == 2;
    } else if(hasCube.getAsBoolean()) {
      match = selectedColumn == 1;
    }
    int selectedGridId = selectedColumn / 3;
    int gridColumnId = selectedColumn % 3;
    gridWidgets[selectedGridId][gridColumnId][selectedRow]
        .withProperties(Map.of("Color When True", match ? Color.kGreen.toHexString() : Color.kRed.toHexString()));
  }

  private void makeBox(final ShuffleboardLayout gridsLayout, final String type, final int gridIndex,
      final int columnIndex, final int rowId, final Color color) {
    var panel = gridsLayout.addBoolean("Cube " + gridIndex + " " + columnIndex + " " + rowId, () -> gridValues[gridIndex][columnIndex][rowId])
        .withProperties(Map.of("Color When True", Color.kGreen.toHexString(), "Color When False", color.toHexString()))
        .withPosition(gridIndex * 3 + columnIndex, rowId);
    gridWidgets[gridIndex][columnIndex][rowId] = panel;
  }

  public void selectRow(int row) {
    this.selectedRow = MathUtil.clamp(row, 0, 2);
    refreshGrid();
  }

  public void selectedColumn(int column) {
    this.selectedColumn = MathUtil.clamp(column, 0, 2);
    refreshGrid();
  }

  private void refreshGrid() {
    int selectedGridId = selectedColumn / 3;
    int gridColumnId = selectedColumn % 3;
    for (int gridId = 0; gridId < 3; gridId++) {
      for (int columnId = 0; columnId < 3; columnId++) {
        for (int rowId = 0; rowId < 3; rowId++) {
          gridValues[gridId][columnId][rowId] = (rowId == selectedRow && selectedGridId == gridId && gridColumnId == columnId);
        }
      }
    }
  }

  public int getSelectedRow() {
    return selectedRow;
  }

  public int getSelectedColumn() {
    return selectedColumn;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
