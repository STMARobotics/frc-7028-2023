package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class JustPickupConeCommand extends CommandBase {

  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;

  private final double elevatorMeters;
  private final double wristRadians;
  private final double intakeDutyCycle;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  private boolean intaking = false;

  public JustPickupConeCommand(
      double elevatorMeters, double wristRadians, double intakeDutyCycle, ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {

    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.intakeDutyCycle = intakeDutyCycle;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    intaking = false;
  }
  
  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(elevatorMeters);
    wristSubsystem.moveToPosition(wristRadians);

    var readyToIntake =
        Math.abs(elevatorSubsystem.getElevatorPosition() - elevatorMeters) < ELEVATOR_TOLERANCE
        && Math.abs(wristSubsystem.getWristPosition() - wristRadians) < WRIST_TOLERANCE;

    if (intaking || readyToIntake) {
      shooterSubsystem.shootDutyCycle(intakeDutyCycle);
    }
  }
  
  @Override
  public boolean isFinished() {
    return shooterSubsystem.hasCone();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

}
