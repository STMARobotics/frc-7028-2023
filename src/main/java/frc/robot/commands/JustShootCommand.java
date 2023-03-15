package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Basic command to position elevator and wrist, and then shoot
 */
public class JustShootCommand extends CommandBase {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double SHOOT_TIME = 0.25;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Timer shootTimer = new Timer();

  private final MedianFilter elevatorFilter = new MedianFilter(5);
  private final MedianFilter wristFilter = new MedianFilter(5);
  private final Debouncer wristDebouncer = new Debouncer(0.1, DebounceType.kRising);

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
  public JustShootCommand(double elevatorMeters, double wristRadians, double shooterRPS,
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem,
      LEDSubsystem ledSubsystem) {
    this(elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;
  }

  protected JustShootCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, 
      ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
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
    wristDebouncer.calculate(false);
    wristSubsystem.moveToPosition(wristRadians);
    elevatorReady = false;
    wristReady = false;
    ledSubsystem.setCustomMode(leds -> leds.setLEDSegments(LEDSubsystem.CUBE_COLOR, elevatorReady, wristReady));
    shooterSubsystem.activeStop();
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(elevatorMeters);

    var elevatorPosition = elevatorFilter.calculate(elevatorSubsystem.getElevatorPosition());
    var wristPosition = wristFilter.calculate(wristSubsystem.getWristPosition());
    elevatorReady = Math.abs(elevatorPosition - elevatorMeters) < ELEVATOR_TOLERANCE;
    wristReady = Math.abs(wristPosition - wristRadians) < WRIST_TOLERANCE;
    var readyToShoot = elevatorReady && wristDebouncer.calculate(wristReady);

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
