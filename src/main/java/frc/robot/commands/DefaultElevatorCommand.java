package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase {

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
