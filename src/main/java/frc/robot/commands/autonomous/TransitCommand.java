package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command for autonomous to put the elevator and wrist into transit position. Similar to what DefaultWristCommand and
 * DefaultElevatorCommand do, but it stops once they reach a safe position to start moving.
 */
public class TransitCommand extends CommandBase {

  private static final double WRIST_SAFE_POSITION = 0.09;
   
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
    wristSubsystem.parkWrist();
    elevatorSubsystem.parkElevator();
  }

  @Override
  public boolean isFinished() {
    return wristSubsystem.getWristPosition() > WRIST_SAFE_POSITION && elevatorSubsystem.isParked();
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    System.out.println("Transit finished");
  }
  
}
