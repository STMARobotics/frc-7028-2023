package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase {

  private static final double ELEVATOR_PARK_HEIGHT = .06;
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final BooleanSupplier idleSupplier;

  public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier idleSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.idleSupplier = idleSupplier;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    if (idleSupplier.getAsBoolean() && (elevatorSubsystem.getElevatorPosition() - ELEVATOR_PARK_HEIGHT) < .01) {
      elevatorSubsystem.stop();
    } else {
      elevatorSubsystem.moveToPosition(ELEVATOR_PARK_HEIGHT);
    }
  }
}
