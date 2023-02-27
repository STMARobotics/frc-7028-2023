package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Mode;

/**
 * Command to run a custom LED mode. This is useful if the mode requires refreshing (like alternate), or if a command
 * is needed to hold the subsystem so the default command doesn't take over.
 */
public class LEDCustomCommand extends CommandBase {
  private final Consumer<LEDSubsystem> periodicRunnable;
  private final LEDSubsystem ledSubsystem;

  /**
   * Constructor
   * @param periodicRunnable method to run in execute() to update the LED subsystem
   * @param ledSubsystem led subsystem
   */
  public LEDCustomCommand(Consumer<LEDSubsystem> periodicRunnable, LEDSubsystem ledSubsystem) {
    this.periodicRunnable = periodicRunnable;
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }
  @Override
  public void initialize() {
    ledSubsystem.setMode(Mode.CUSTOM);
  }

  @Override
  public void execute() {
    periodicRunnable.accept(ledSubsystem);
  }

}
