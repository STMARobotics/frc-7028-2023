package frc.robot.commands;

import static frc.robot.Constants.ConeShootingConstants.SHOOT_TIME;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Basic command to position elevator and wrist, and then shoot
 */
public class JustShootCommand extends CommandBase {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final Timer shootTimer = new Timer();

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
  public JustShootCommand(double elevatorMeters, double wristRadians, double shooterRPS, ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem);
  }

  protected JustShootCommand(ElevatorSubsystem elevatorSubsystem,
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
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(elevatorMeters);
    wristSubsystem.moveToPosition(wristRadians);

    var readyToShoot =
        Math.abs(elevatorSubsystem.getElevatorPosition() - elevatorMeters) < ELEVATOR_TOLERANCE
        && Math.abs(wristSubsystem.getWristPosition() - wristRadians) < WRIST_TOLERANCE;

    if (isShooting || readyToShoot) {
      shooterSubsystem.shootVelocity(shooterRPS);
      shootTimer.start();
      isShooting = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isShooting && shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    wristSubsystem.stop();
    shooterSubsystem.stop();
  }

}
