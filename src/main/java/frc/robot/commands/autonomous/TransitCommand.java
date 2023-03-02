package frc.robot.commands.autonomous;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT;
import static frc.robot.Constants.WristConstants.WRIST_PARK_HEIGHT;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command for autonomous while in transit. Similar to what DefaultWristCommand and DefaultElevatorCommand do, but it
 * stops once they reach their park positions.
 */
public class TransitCommand extends CommandBase {
   
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public TransitCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem);
  }
  
  @Override
  public void initialize() {
    shooterSubsystem.activeStop();
  }

  @Override
  public void execute() {
    wristSubsystem.moveToPosition(WRIST_PARK_HEIGHT);
    elevatorSubsystem.moveToPosition(ELEVATOR_PARK_HEIGHT);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(wristSubsystem.getWristPosition() - WRIST_PARK_HEIGHT) < 0.2
        && Math.abs(elevatorSubsystem.getElevatorPosition() - ELEVATOR_PARK_HEIGHT) < 0.2;
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }
  
}
