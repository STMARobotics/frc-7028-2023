package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Basic command to position elevator and wrist, and then shoot
 */
public class JustShootCommand extends Command {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double SHOOT_TIME = 0.25;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Timer shootTimer = new Timer();

  private final MedianFilter elevatorFilter = new MedianFilter(5);
  private final Color ledColor;

  protected double elevatorMeters;
  protected double wristRadians;
  protected double shooterRPS;
  
  private boolean isShooting = false;
  private boolean elevatorReady = false;
  private boolean wristReady = false;

  /**
   * Constructor
   * @param elevatorMeters position of elevator in meters
   * @param wristRadians position of wrist in radians
   * @param shooterRPS velocity of shooter in rotations per second
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   */
  public JustShootCommand(double elevatorMeters, double wristRadians, double shooterRPS, Color ledColor,
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem,
      LEDSubsystem ledSubsystem) {
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;
    this.ledColor = ledColor;

    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    isShooting = false;
    wristSubsystem.moveToPosition(wristRadians);
    elevatorReady = false;
    wristReady = false;
    ledSubsystem.setCustomMode(leds -> leds.setLEDSegments(ledColor, elevatorReady, wristReady));
    shooterSubsystem.activeStop();
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(elevatorMeters);

    var elevatorPosition = elevatorFilter.calculate(elevatorSubsystem.getElevatorPosition());
    elevatorReady = Math.abs(elevatorPosition - elevatorMeters) < ELEVATOR_TOLERANCE;
    wristReady = Math.abs(wristSubsystem.getWristPosition() - wristRadians) < WRIST_TOLERANCE;
    var readyToShoot = elevatorReady && wristReady;

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
    if (isShooting) {
      shooterSubsystem.stop();
    } else {
      shooterSubsystem.activeStop();
    }
    shootTimer.stop();
  }

}
