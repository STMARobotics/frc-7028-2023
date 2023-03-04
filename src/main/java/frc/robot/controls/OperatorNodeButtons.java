package frc.robot.controls;

public enum OperatorNodeButtons {
  NODE_TOP_LEFT(0, 7, 0, 0),
  NODE_TOP_CENTER(0, 8, 1, 0),
  NODE_TOP_RIGHT(0, 9, 2 ,0),
  NODE_MID_LEFT(0, 10, 0, 1),
  NODE_MID_CENTER(1, 1, 1, 1),
  NODE_MID_RIGHT(1, 2, 2, 1),
  NODE_BOTTOM_LEFT(1, 3, 0, 2),
  NODE_BOTTOM_CENTER(1, 4, 1, 2),
  NODE_BOTTOM_RIGHT(1, 5, 2, 2);
  
  public final int joystickId;
  public final int buttonId;
  public final int column;
  public final int row;

  private OperatorNodeButtons(int joystickId, int buttonId, int column, int row) {
    this.joystickId = joystickId;
    this.buttonId = buttonId;
    this.column = column;
    this.row = row;
  }


}
