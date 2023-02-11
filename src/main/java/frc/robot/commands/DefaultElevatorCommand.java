package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command that raises the elevator if grabber has a cube
 */
public class DefaultElevatorCommand extends CommandBase {

  private final ElevatorSubsystem elevatorSubsystem;
  private final BooleanSupplier raiseElevator;

  public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier raiseElevator) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.raiseElevator = raiseElevator;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    if (raiseElevator.getAsBoolean()) {
      elevatorSubsystem.moveToPosition(0.450);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }
}
