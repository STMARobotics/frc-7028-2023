package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreLocationSubsystem extends SubsystemBase {

  // Array of values [grid][column][row]
  private final boolean[][][] gridValues = new boolean[3][3][3];
  
  // Array of widgets [grid][column][row]
  @SuppressWarnings("unchecked")
  private final SuppliedValueWidget<Boolean>[][][] gridWidgets = new SuppliedValueWidget[3][3][3];
  private final BooleanSupplier hasCone;
  private final BooleanSupplier hasCube;

  private int selectedRow = 2;
  private int selectedColumn = 1;
  private int selectedGrid = 1;
  
  public ScoreLocationSubsystem(BooleanSupplier hasCone, BooleanSupplier hasCube) {
    this.hasCone = hasCone;
    this.hasCube = hasCube;

    var gridsTab = Shuffleboard.getTab("Grids");
    var gridsLayout = gridsTab.getLayout("Grids", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 9, "Number of rows", 3, "Label Position", "HIDDEN"))
        .withSize(9, 3)
        .withPosition(0, 0);

    for(int gridId = 0; gridId < 3; gridId++) {
      for(int columnId = 0; columnId < 3; columnId++) {
        for (int rowId = 0; rowId < 3; rowId++) {
          makeWidget(gridsLayout, gridId, columnId, rowId, 1 == columnId ? Color.kPurple : Color.kYellow);
        }
      }
    }
  }

  @Override
  public void periodic() {
    setSelectedTrue();
    // check for mismatch
    boolean match = true;
    if (selectedRow == 0) {
      match = true;
    } else if(hasCone.getAsBoolean()) {
      match = selectedColumn == 0 || selectedColumn == 2;
    } else if(hasCube.getAsBoolean()) {
      match = selectedColumn == 1;
    }
    gridWidgets[selectedGrid][selectedColumn][selectedRow]
        .withProperties(Map.of("Color When True", match ? Color.kGreen.toHexString() : Color.kRed.toHexString(),
            "Color When False", selectedColumn == 1 ? Color.kPurple.toHexString() : Color.kYellow.toHexString()));
  }

  private void makeWidget(ShuffleboardLayout gridsLayout, int gridIndex, int columnIndex, int rowId, Color color) {
    var panel = gridsLayout.addBoolean(gridIndex + " " + columnIndex + " " + rowId, () -> gridValues[gridIndex][columnIndex][rowId])
        .withProperties(Map.of("Color When True", Color.kGreen.toHexString(), "Color When False", color.toHexString()))
        .withPosition(gridIndex * 3 + columnIndex, rowId);
    gridWidgets[gridIndex][columnIndex][rowId] = panel;
  }

  public ScoreLocationSubsystem selectGrid(int grid) {
    setSelectedFalse();
    this.selectedGrid = MathUtil.clamp(grid, 0, 2);
    setSelectedTrue();
    return this;
  }

  public ScoreLocationSubsystem selectColumn(int column) {
    setSelectedFalse();
    this.selectedColumn = MathUtil.clamp(column, 0, 2);
    setSelectedTrue();
    return this;
  }

  public ScoreLocationSubsystem selectRow(int row) {
    setSelectedFalse();
    this.selectedRow = MathUtil.clamp(row, 0, 2);
    setSelectedTrue();
    return this;
  }

  private void setSelectedFalse() {
    gridValues[selectedGrid][selectedColumn][selectedRow] = false;
  }

  private void setSelectedTrue() {
    gridValues[selectedGrid][selectedColumn][selectedRow] = true;
  }

  public int getSelectedGrid() {
    return selectedGrid;
  }

  public int getSelectedColumn() {
    return selectedColumn;
  }

  public int getSelectedRow() {
    return selectedRow;
  }

}
