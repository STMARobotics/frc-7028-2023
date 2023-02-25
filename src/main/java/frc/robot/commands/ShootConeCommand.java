package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.limelight.LimelightProfile;
import frc.robot.limelight.VisionTargetInfo;
import frc.robot.math.MovingAverageFilter;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Mode;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterProfile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to drive within range, turn to target, position elevator and wrist, and then shoot
 */
public class ShootConeCommand extends CommandBase {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double DISTANCE_TOLERANCE = 0.1;
  private static final double SHOOT_TIME = 5.0;
  private static final double AIM_OFFSET = -0.01;

  private static final TrapezoidProfile.Constraints DISTANCE_CONSTRAINTS = new TrapezoidProfile.Constraints(2.0, 4.0);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI);

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Timer shootTimer = new Timer();

  private final ProfiledPIDController aimController = new ProfiledPIDController(3.5, 0.0, 0, OMEGA_CONSTRAINTS);
  private final ProfiledPIDController distanceController = new ProfiledPIDController(1.5, 0, 0, DISTANCE_CONSTRAINTS);

  private final LimelightProfile limelightProfile;
  private final ShooterProfile shooterProfile;
  private final LimelightCalcs limelightCalcs;

  private final MedianFilter elevatoFilter = new MedianFilter(5);
  private final MedianFilter wristFilter = new MedianFilter(5);
  private final Debouncer readyToShootDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private final MovingAverageFilter distanceFilter = new MovingAverageFilter(3);
  private final MovingAverageFilter rotationFilter = new MovingAverageFilter(3);

  private VisionTargetInfo lastTargetInfo = null;
  private boolean isShooting = false;

  /**
   * Constructor
   * @param shooterProfile shooter profile
   * @param limelightProfile limelight profile
   * @param drivetrainSubsystem drivetrain
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   * @param limelightSubssystem limelight
   * @param ledSubsystem LED subsystem
   */
  public ShootConeCommand(ShooterProfile shooterProfile, LimelightProfile limelightProfile,
      DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, LEDSubsystem ledSubsystem) {
    this.shooterProfile = shooterProfile;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.limelightProfile = limelightProfile;

    limelightCalcs = new LimelightCalcs(limelightProfile.cameraToRobot, limelightProfile.targetHeight, 
        limelightProfile.cameraOnElevator ? elevatorSubsystem::getElevatorTopPosition : () -> 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    distanceController.setGoal(shooterProfile.shootingDistance);

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    lastTargetInfo = null;
    isShooting = false;
    aimController.reset(drivetrainSubsystem.getGyroscopeRotation().getRadians());
    aimController.setTolerance(shooterProfile.aimTolerance / 3.0);
    distanceController.setTolerance(DISTANCE_TOLERANCE / 2.0);
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(limelightProfile.pipelineId);
    readyToShootDebouncer.calculate(false);
    elevatoFilter.reset();
    wristFilter.reset();
    distanceFilter.reset();
    rotationFilter.reset();
  }

  @Override
  public void execute() {
    var firstTarget = false;
    // If the target is visible, get the new translation. If the target isn't visible we'll use the last known translation.
    var limelightRetroResults = limelightSubsystem.getLatestRetroTarget();
    if (limelightRetroResults.isPresent()) {
      firstTarget = lastTargetInfo == null;
      lastTargetInfo = limelightCalcs.getRobotRelativeTargetInfo(limelightRetroResults.get());
    }

    if (lastTargetInfo == null) {
      // We've never seen a target
      elevatorSubsystem.stop();
      wristSubsystem.stop();
      shooterSubsystem.stop();
      drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 1.0));
      ledSubsystem.setMode(Mode.SHOOTING_NO_TARGET);
    } else {
      // Get shooter settings from lookup table
      var shooterSettings = shooterProfile.lookupTable.calculate(lastTargetInfo.distance);

      // Get the robot heading, and the robot-relative heading of the target
      var drivetrainHeading = drivetrainSubsystem.getGyroscopeRotation();
      var targetHeading = drivetrainHeading.plus(lastTargetInfo.angle);
      
      if (firstTarget) {
        // On the first iteration, reset the PID controller
        distanceController.reset(lastTargetInfo.distance);
      }

      // Get corrections from PID controllers
      aimController.setGoal(targetHeading.getRadians() + AIM_OFFSET);
      var rotationCorrection = aimController.calculate(drivetrainHeading.getRadians());
      var distanceCorrection = -distanceController.calculate(lastTargetInfo.distance);

      // Filter the elevator and wrist positions, to prevent oscillation from mechanical shake
      var elevatorPosition = elevatoFilter.calculate(elevatorSubsystem.getElevatorPosition());
      var wristPosition = wristFilter.calculate(wristSubsystem.getWristPosition());

      // Check if the elevator and wrist are in the right position, and if the distance and angle are right
      var readyToShoot = readyToShootDebouncer.calculate(
          Math.abs(elevatorPosition - shooterSettings.height) < ELEVATOR_TOLERANCE
          && Math.abs(wristPosition - shooterSettings.angle) < WRIST_TOLERANCE
          && Math.abs(lastTargetInfo.angle.getRadians()) < shooterProfile.aimTolerance
          && Math.abs(lastTargetInfo.distance - shooterProfile.shootingDistance) < DISTANCE_TOLERANCE);

      SmartDashboard.putBoolean("Elevator", Math.abs(elevatorPosition - shooterSettings.height) < ELEVATOR_TOLERANCE);
      SmartDashboard.putBoolean("Wrist", Math.abs(wristPosition - shooterSettings.angle) < WRIST_TOLERANCE);
      SmartDashboard.putBoolean("Aim",  Math.abs(lastTargetInfo.angle.getRadians()) < shooterProfile.aimTolerance);
      SmartDashboard.putBoolean("Distance", Math.abs(lastTargetInfo.distance - shooterProfile.shootingDistance) < DISTANCE_TOLERANCE);

      if (isShooting || readyToShoot) {
        // Shoot
        ledSubsystem.setMode(Mode.SHOOTING);
        shooterSubsystem.shootVelocity(shooterSettings.velocity);
        shootTimer.start();
        isShooting = true;
        drivetrainSubsystem.stop();
      } else {
        ledSubsystem.setMode(Mode.SHOOTING_HAS_TARGET);
        // Not ready to shoot, move the elevator, wrist, and drivetrain
        elevatorSubsystem.moveToPosition(shooterSettings.height);
        wristSubsystem.moveToPosition(shooterSettings.angle);

        // Rotate the distance measurement so we drive toward the target in X and Y direction, not just robot forward
        var xySpeeds = new Translation2d(distanceFilter.calculate(distanceCorrection), 0).rotateBy(lastTargetInfo.angle);
        drivetrainSubsystem.drive(
            new ChassisSpeeds(xySpeeds.getX(), xySpeeds.getY(), rotationFilter.calculate(rotationCorrection)));
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
    drivetrainSubsystem.stop();
  }

}