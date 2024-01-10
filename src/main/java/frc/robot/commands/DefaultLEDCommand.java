package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Mode;

/**
 * Default LED command to make the LEDs dance or go yellow when holding a cone.
 */
public class DefaultLEDCommand extends Command {

  private final LEDSubsystem ledSubsystem;
  private final BooleanSupplier hasCone;
  private final BooleanSupplier hasCube;

  public DefaultLEDCommand(LEDSubsystem ledSubsystem, BooleanSupplier hasCone, BooleanSupplier hasCube) {
    this.ledSubsystem = ledSubsystem;
    this.hasCone = hasCone;
    this.hasCube = hasCube;

    addRequirements(ledSubsystem);
  }

  @Override
  public void execute() {
    if (!DriverStation.isDSAttached()) {
      ledSubsystem.setMode(Mode.DS_DISCONNECT);
    } else if (hasCone.getAsBoolean()) {
      ledSubsystem.setMode(Mode.HAS_CONE);
    } else if (hasCube.getAsBoolean()) {
      ledSubsystem.setMode(Mode.HAS_CUBE);
    } else if (RobotState.isDisabled()) {
      ledSubsystem.setMode(Mode.BLUE_GOLD);
    } else {
      ledSubsystem.setMode(Mode.OFF);
    }
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
  
}
