package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PickupPosition extends CommandBase {
  private final double elevatorMeters;
  private final double wristRadians;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;

  public PickupPosition(
      double elevatorMeters, double wristRadians,
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {

    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem);
  }
  
  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(elevatorMeters);
    wristSubsystem.moveToPosition(wristRadians);
    }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    wristSubsystem.stop();
  }
}
