package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Mode;
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
  private final Debouncer readyToShootDebouncer = new Debouncer(.25, DebounceType.kRising);

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
    readyToShootDebouncer.calculate(false);
    wristSubsystem.moveToPosition(wristRadians);
    elevatorReady = false;
    wristReady = false;
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(elevatorMeters);

    var elevatorPosition = elevatorFilter.calculate(elevatorSubsystem.getElevatorPosition());
    var wristPosition = wristFilter.calculate(wristSubsystem.getWristPosition());
    elevatorReady = Math.abs(elevatorPosition - elevatorMeters) < ELEVATOR_TOLERANCE;
    wristReady = Math.abs(wristPosition - wristRadians) < WRIST_TOLERANCE;
    var readyToShoot = readyToShootDebouncer.calculate(elevatorReady && wristReady);
    updateReadyStateLEDs();

    if (isShooting || readyToShoot) {
      shooterSubsystem.shootVelocity(shooterRPS);
      shootTimer.start();
      isShooting = true;
    }
  }

  /**
   * Updates the LED strips. 1/2 of each strip indicates a status: elevator, wrist
   */
  private void updateReadyStateLEDs() {
    boolean[] statuses = new boolean[] {elevatorReady, wristReady};
    int ledsPerStatus = LEDSubsystem.STRIP_SIZE / statuses.length;
    ledSubsystem.setMode(Mode.CUSTOM);
    for(int stripId = 0; stripId < LEDSubsystem.STRIP_COUNT; stripId++) {
      int ledIndex = 0;
      for (int statusId = 0; statusId < statuses.length; statusId++) {
        for(;ledIndex < (ledsPerStatus * (statusId + 1)); ledIndex++) {
          ledSubsystem.setLED(stripId, ledIndex, statuses[statusId] ? LEDSubsystem.CUBE_COLOR : Color.kBlack);
        }
      }
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
    shootTimer.stop();
  }

}
