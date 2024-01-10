package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to run a custom LED mode. This is useful if the mode requires refreshing (like alternate), or if a command
 * is needed to hold the subsystem so the default command doesn't take over.
 */
public class LEDCustomCommand extends Command {
  private final Consumer<LEDStrips> periodicRunnable;
  private final LEDSubsystem ledSubsystem;

  /**
   * Constructor
   * @param periodicRunnable method to update the LED strips
   * @param ledSubsystem led subsystem
   */
  public LEDCustomCommand(Consumer<LEDStrips> periodicRunnable, LEDSubsystem ledSubsystem) {
    this.periodicRunnable = periodicRunnable;
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }
  
  @Override
  public void initialize() {
    ledSubsystem.setCustomMode(periodicRunnable);
  }

}
