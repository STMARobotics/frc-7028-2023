package frc.robot.controls;

public enum OperatorButtons {
  
  GRID_LEFT(1, 6),
  GRID_CENTER(1, 7),
  GRID_RIGHT(1, 8),
  ENTER_SELECTION(1, 10);

  public final int joystickId;
  public final int buttonId;

  private OperatorButtons(int joystickId, int buttonId) {
    this.joystickId = joystickId;
    this.buttonId = buttonId;
  }

}
