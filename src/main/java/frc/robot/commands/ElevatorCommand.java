package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final DoubleSupplier upSupplier;
  private final DoubleSupplier downSupplier;

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier upSupplier, DoubleSupplier downSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.upSupplier = upSupplier;
    this.downSupplier = downSupplier;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    double upSpeed = upSupplier.getAsDouble();
    double downSpeed = downSupplier.getAsDouble();

    if (upSpeed > 0 && downSpeed > 0) {
      elevatorSubsystem.stop();
    } else if (upSpeed > 0) {
      elevatorSubsystem.elevatorUp(upSpeed);
    } else if (downSpeed > 0) {
      elevatorSubsystem.elevatorDown(-downSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

}
