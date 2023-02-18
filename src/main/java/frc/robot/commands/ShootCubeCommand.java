package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootCubeCommand extends CommandBase {
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final Timer shootTimer = new Timer();

  private final MedianFilter elevatoFilter = new MedianFilter(5);
  private final MedianFilter wristFilter = new MedianFilter(5);
  private final Debouncer readyToShootDebouncer = new Debouncer(.25, DebounceType.kRising);

  protected double elevatorMeters;
  protected double wristRadians;
  protected double shooterRPS;
  
  private boolean isShooting = false;

  /**
   * Constructor
   * @param elevatorMeters position of elevator in meters
   * @param wristRadians position of wrist in radians
   * @param shooterRPS velocity of shooter in rotations per second
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   */
  public ShootCubeCommand(double elevatorMeters, double wristRadians, double shooterRPS, ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem);
  }

  protected ShootCubeCommand(ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    isShooting = false;
    readyToShootDebouncer.calculate(false);
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(elevatorMeters);
    wristSubsystem.moveToPosition(wristRadians);

    var elevatorPosition = elevatoFilter.calculate(elevatorSubsystem.getElevatorPosition());
    var wristPosition = wristFilter.calculate(wristSubsystem.getWristPosition());
    var readyToShoot = readyToShootDebouncer.calculate(
        Math.abs(elevatorPosition - elevatorMeters) < ELEVATOR_TOLERANCE
        && Math.abs(wristPosition - wristRadians) < WRIST_TOLERANCE);

    if (isShooting || readyToShoot) {
      shooterSubsystem.shootVelocity(shooterRPS);
      shootTimer.start();
      if (!isShooting) {
        System.out.println("Elevator: " + elevatorSubsystem.getElevatorPosition());
        System.out.println("Wrist: " + wristSubsystem.getWristPosition());
        System.out.println("Shooter: " + shooterSubsystem.getVelocity());
      }
      isShooting = true;
    }
  }

  @Override
  public boolean isFinished() {
    //return false;
    return isShooting && shootTimer.hasElapsed(3);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    wristSubsystem.stop();
    shooterSubsystem.stop();
  }
}