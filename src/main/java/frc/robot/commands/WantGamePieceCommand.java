package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GamePiece;
import frc.robot.subsystems.LEDSubsystem;

public class WantGamePieceCommand extends CommandBase {
  private final LEDSubsystem ledSubsystem;
  private GamePiece gamePiece;

  public WantGamePieceCommand(LEDSubsystem ledSubsystem, GamePiece gamePiece) {
    this.ledSubsystem = ledSubsystem;
    this.gamePiece = gamePiece;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    ledSubsystem.setMode(gamePiece.ledMode);
  }
}
