package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MovingAverageFilter;
import frc.robot.limelight.LimelightRetroCalcs;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to drive within range, turn to target, position elevator and wrist, and then shoot
 */
public class InterpolateShootCommand extends CommandBase {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double AIM_TOLERANCE = Units.degreesToRadians(1.0);
  private static final double DISTANCE_TOLERANCE = 0.025;
  private static final double DISTANCE_GOAL = 1.10d;

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final Timer shootTimer = new Timer();

  private final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI);
  private final ProfiledPIDController aimController = new ProfiledPIDController(1.5, 0, 0, kThetaControllerConstraints);
  private final PIDController distanceController = new PIDController(1.5, 0, 0);

  private final Profile shooterProfile;
  private final LimelightRetroCalcs limelightCalcs;

  private final MedianFilter elevatoFilter = new MedianFilter(5);
  private final MedianFilter wristFilter = new MedianFilter(5);
  private final Debouncer readyToShootDebouncer = new Debouncer(.25, DebounceType.kRising);
  private final MovingAverageFilter distanceFilter = new MovingAverageFilter(5);

  private boolean isShooting = false;

  /**
   * Constructor
   * @param shooterProfile shooter profile
   * @param drivetrainSubsystem drivetrain
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   * @param limelightSubssystem limelight
   * @param cameraToRobot camera to robot transformation
   */
  public InterpolateShootCommand(Profile shooterProfile,
      DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterProfile = shooterProfile;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    limelightCalcs = new LimelightRetroCalcs(shooterProfile.cameraToRobot, shooterProfile.targetHeight);
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    isShooting = false;
    aimController.reset(drivetrainSubsystem.getGyroscopeRotation().getRadians());
    aimController.setTolerance(AIM_TOLERANCE);
    limelightSubsystem.enable();
    distanceController.reset();
    distanceController.setSetpoint(DISTANCE_GOAL);
    distanceController.setTolerance(DISTANCE_TOLERANCE);
    limelightSubsystem.setPipelineId(shooterProfile.pipelineId);
    readyToShootDebouncer.calculate(false);
    elevatoFilter.reset();
    wristFilter.reset();
    distanceFilter.reset();
  }

  @Override
  public void execute() {
    limelightSubsystem.getLatestRetroTarget().ifPresentOrElse((limelightRetroResults) -> {
      
      // get the distance and angle of the target, relative to the robot
      var targetPose = limelightCalcs.getTargetPose(limelightRetroResults);
      var targetDistance = distanceFilter.calculate(targetPose.getTranslation().getDistance(new Translation2d()));
      var targetAngle = new Rotation2d(targetPose.getX(), targetPose.getY());

      var shooterSettings = shooterProfile.lookupTable.calculate(targetDistance);

      var drivetrainHeading = drivetrainSubsystem.getGyroscopeRotation();
      var targetHeadingRadians = drivetrainHeading.minus(targetAngle).getRadians();
      
      aimController.setGoal(targetHeadingRadians);
      var rotationCorrection = 
          Math.abs(targetAngle.getRadians()) > AIM_TOLERANCE ? -aimController.calculate(drivetrainHeading.getRadians()) : 0;
      var distanceCorrection = 
          Math.abs(targetDistance - DISTANCE_GOAL) > DISTANCE_TOLERANCE ? -distanceController.calculate(targetDistance) : 0;

      var elevatorPosition = elevatoFilter.calculate(elevatorSubsystem.getElevatorPosition());
      var wristPosition = wristFilter.calculate(wristSubsystem.getWristPosition());

      var readyToShoot = readyToShootDebouncer.calculate(
          Math.abs(elevatorPosition - shooterSettings.height) < ELEVATOR_TOLERANCE
          && Math.abs(wristPosition - shooterSettings.angle) < WRIST_TOLERANCE
          && rotationCorrection == 0
          && distanceCorrection == 0);

      if (isShooting || readyToShoot) {
        shooterSubsystem.shootVelocity(shooterSettings.velocity);
        shootTimer.start();
        isShooting = true;
        drivetrainSubsystem.stop();
      } else {
        elevatorSubsystem.moveToPosition(shooterSettings.height);
        wristSubsystem.moveToPosition(shooterSettings.angle);
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
