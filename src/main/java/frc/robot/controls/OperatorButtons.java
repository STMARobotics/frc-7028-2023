package frc.robot.controls;

public enum OperatorButtons {
  
  GRID_LEFT(0, 1),
  GRID_CENTER(0, 2),
  GRID_RIGHT(0, 3),
  MODE_CUBE(1, 1),
  MODE_CONE(1, 2),
  ENTER_SELECTION(1, 3);

  public final int joystickId;
  public final int buttonId;

  private OperatorButtons(int joystickId, int buttonId) {
    this.joystickId = joystickId;
    this.buttonId = buttonId;
  }

}
