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
    upSpeed = Math.copySign(upSpeed * upSpeed, upSpeed);

    double downSpeed = downSupplier.getAsDouble();
    downSpeed = Math.copySign(downSpeed * downSpeed, downSpeed);
    
    if (upSpeed > 0) {
      elevatorSubsystem.elevatorUp(upSpeed);
    } else if (downSpeed > 0) {
      elevatorSubsystem.elevatorDown(-downSpeed);
    } else {
      elevatorSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

}
