package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoIntakeCommand extends CommandBase {
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;

  private final double elevatorMeters;
  private final double wristRadians;
  private final double intakeDutyCycle;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  
  private final Timer pickupTimer = new Timer();

  private boolean intaking = false;

  public AutoIntakeCommand(
      double elevatorMeters, double wristRadians, double intakeDutyCycle,
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem) {

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
    pickupTimer.reset();
    intaking = false;
  }
  
  @Override
  public void execute() {
    pickupTimer.start();
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
    return shooterSubsystem.hasCone() || pickupTimer.hasElapsed(5);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    elevatorSubsystem.stop();
    wristSubsystem.stop();
  }
}
