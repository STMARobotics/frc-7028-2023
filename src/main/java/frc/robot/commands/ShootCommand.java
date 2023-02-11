package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to drive within range, turn to target, position elevator and wrist, and then shoot
 */
public class ShootCommand extends CommandBase {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double AIM_TOLERANCE = 1.0;
  private static final double DISTANCE_TOLERANCE = 0.15;
  private static final double TARGET_Y_SETPOINT = 12.83; // degrees in the limelight top target

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final Timer shootTimer = new Timer();

  private final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);
  private final ProfiledPIDController aimController = new ProfiledPIDController(2.0, 0, 0, kThetaControllerConstraints);
  private final PIDController distanceController = new PIDController(.5, 0, 0);

  private final Profile shooterProfile;
  private final double elevatorMeters;
  private final double wristRadians;
  private final double shooterRPS;

  private final MedianFilter elevatoFilter = new MedianFilter(5);
  private final MedianFilter wristFilter = new MedianFilter(5);
  private final Debouncer readyToShootDebouncer = new Debouncer(.25, DebounceType.kRising);

  private boolean isShooting = false;

  /**
   * Constructor
   * @param elevatorMeters position of elevator in meters
   * @param wristRadians position of wrist in radians
   * @param shooterRPS velocity of shooter in rotations per second
   * @param shooterProfile limelight profile
   * @param drivetrainSubsystem drivetrain
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   * @param limelightSubssystem limelight
   */
  public ShootCommand(double elevatorMeters, double wristRadians, double shooterRPS, Profile shooterProfile,
      DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;
    this.shooterProfile = shooterProfile;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    isShooting = false;
    aimController.reset(drivetrainSubsystem.getGyroscopeRotation().getRadians());
    distanceController.setSetpoint(TARGET_Y_SETPOINT);
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(shooterProfile.pipelineId);
    readyToShootDebouncer.calculate(false);
  }

  @Override
  public void execute() {
    limelightSubsystem.getLatestRetroTarget().ifPresentOrElse((limelightRetroResults) -> {

      var targetX =  limelightRetroResults.targetXDegrees;
      var targetY = limelightRetroResults.targetYDegrees;

      elevatorSubsystem.moveToPosition(elevatorMeters);
      wristSubsystem.moveToPosition(wristRadians);

      var drivetrainHeading = drivetrainSubsystem.getGyroscopeRotation();
      var targetHeadingRadians = drivetrainHeading.getRadians() - Units.degreesToRadians(targetX);
      
      aimController.setGoal(targetHeadingRadians);
      var rotationCorrection = 
          Math.abs(targetX) > AIM_TOLERANCE ? aimController.calculate(drivetrainHeading.getRadians()) : 0;
      var distanceCorrection =
          Math.abs(targetY - TARGET_Y_SETPOINT) > DISTANCE_TOLERANCE ? distanceController.calculate(targetY) : 0;

      var elevatorPosition = elevatoFilter.calculate(elevatorSubsystem.getElevatorPosition());
      var wristPosition = wristFilter.calculate(wristSubsystem.getWristPosition());
      var readyToShoot =
          Math.abs(elevatorPosition - elevatorMeters) < ELEVATOR_TOLERANCE
          && Math.abs(wristPosition - wristRadians) < WRIST_TOLERANCE
          && Math.abs(targetX) < AIM_TOLERANCE
          && Math.abs(targetY - TARGET_Y_SETPOINT) < DISTANCE_TOLERANCE;

      if (isShooting || readyToShoot) {
        shooterSubsystem.shootVelocity(shooterRPS);
        shootTimer.start();
        isShooting = true;
        drivetrainSubsystem.stop();
      } else {
        drivetrainSubsystem.drive(new ChassisSpeeds(distanceCorrection, 0, rotationCorrection));
      }
    }, () -> {
      // No target
      elevatorSubsystem.stop();
      wristSubsystem.stop();
      shooterSubsystem.stop();
      drivetrainSubsystem.stop();
    });
  }

  @Override
  public boolean isFinished() {
    return false;
    // return isShooting && shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
    wristSubsystem.stop();
    shooterSubsystem.stop();
    drivetrainSubsystem.stop();
  }

}
