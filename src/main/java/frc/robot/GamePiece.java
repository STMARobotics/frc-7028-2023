package frc.robot;

import static frc.robot.subsystems.LEDSubsystem.Mode.WANT_CONE;
import static frc.robot.subsystems.LEDSubsystem.Mode.WANT_CUBE;

import frc.robot.subsystems.LEDSubsystem.Mode;

/**
 * Game piece types and their associated LED mode
 */
public enum GamePiece {
  CONE(WANT_CONE),
  CUBE(WANT_CUBE);

  public final Mode ledMode;

  private GamePiece(Mode ledMode) {
    this.ledMode = ledMode;
  }
}