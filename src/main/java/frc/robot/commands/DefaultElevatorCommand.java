package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final BooleanSupplier idleSupplier;

  public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier idleSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.idleSupplier = idleSupplier;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    if (idleSupplier.getAsBoolean() && elevatorSubsystem.isParked()) {
      elevatorSubsystem.stop();
    } else {
      elevatorSubsystem.parkElevator();
    }
  }
}
