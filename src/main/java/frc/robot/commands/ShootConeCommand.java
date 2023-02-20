package frc.robot.commands;

import java.util.function.Supplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.limelight.VisionTargetInfo;
import frc.robot.math.MovingAverageFilter;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Mode;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to drive within range, turn to target, position elevator and wrist, and then shoot
 */
public class ShootConeCommand extends DriveToPoseCommand {
  
  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double DISTANCE_GOAL = 1.47;
  private static final double SHOOT_TIME = 0.5;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final LEDSubsystem ledSubsystem;
  private final Timer shootTimer = new Timer();

  private final Profile shooterProfile;
  private final LimelightCalcs limelightCalcs;

  private final MedianFilter elevatoFilter = new MedianFilter(5);
  private final MedianFilter wristFilter = new MedianFilter(5);
  private final Debouncer readyToShootDebouncer = new Debouncer(.25, DebounceType.kRising);
  private final MovingAverageFilter distanceFilter = new MovingAverageFilter(5);

  private VisionTargetInfo lastTargetInfo = null;
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
  public ShootConeCommand(Profile shooterProfile, DrivetrainSubsystem drivetrainSubsystem,
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, Supplier<Pose2d> robotPoseSupplier,
      LEDSubsystem ledSubsystem) {
    super(drivetrainSubsystem, robotPoseSupplier, null);

    this.shooterProfile = shooterProfile;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.robotPoseSupplier = robotPoseSupplier;
    this.ledSubsystem = ledSubsystem;

    DoubleSupplier cameraHeightOffset = 
        shooterProfile.cameraOnElevator ? elevatorSubsystem::getElevatorPosition : () -> 0.0;
    limelightCalcs = new LimelightCalcs(shooterProfile.cameraToRobot, shooterProfile.targetHeight, cameraHeightOffset);


    addRequirements(drivetrainSubsystem, elevatorSubsystem, wristSubsystem, shooterSubsystem, limelightSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    lastTargetInfo = null;
    isShooting = false;
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(shooterProfile.pipelineId);
    readyToShootDebouncer.calculate(false);
    elevatoFilter.reset();
    wristFilter.reset();
    distanceFilter.reset();
    super.initialize();
  }

  @Override
  public void execute() {
    // If the target is visible, get the new translation. If the target isn't visible we'll use the last known translation.
    var limelightRetroResults = limelightSubsystem.getLatestRetroTarget();
    if (limelightRetroResults.isPresent()) {
      lastTargetInfo = limelightCalcs.getRobotRelativeTargetInfo(limelightRetroResults.get());
    }

    if (lastTargetInfo == null) {
      // We've never seen a target
      elevatorSubsystem.stop();
      wristSubsystem.stop();
      shooterSubsystem.stop();
      setGoalPose(null);
      ledSubsystem.setMode(Mode.SHOOTING_NO_TARGET);
    } else {
      ledSubsystem.setMode(Mode.SHOOTING_HAS_TARGET);
      // Get shooter settings from lookup table
      var shooterSettings = shooterProfile.lookupTable.calculate(lastTargetInfo.distance);

      // Get the robot heading, and the robot-relative heading of the target
      var drivetrainHeading = robotPoseSupplier.get().getRotation();
      var targetHeading = drivetrainHeading.minus(lastTargetInfo.angle);

      var goalPose =
          new Pose2d(new Translation2d(lastTargetInfo.distance - DISTANCE_GOAL, targetHeading), targetHeading);
      setGoalPose(goalPose);

      // Filter the elevator and wrist positions, to prevent oscillation from mechanical shake
      var elevatorPosition = elevatoFilter.calculate(elevatorSubsystem.getElevatorPosition());
      var wristPosition = wristFilter.calculate(wristSubsystem.getWristPosition());

      // Check if the elevator and wrist are in the right position, and if the distance and angle are right
      var readyToShoot = readyToShootDebouncer.calculate(
          Math.abs(elevatorPosition - shooterSettings.height) < ELEVATOR_TOLERANCE
          && Math.abs(wristPosition - shooterSettings.angle) < WRIST_TOLERANCE
          && super.atGoal());

      if (isShooting || readyToShoot) {
        // Shoot
        shooterSubsystem.shootVelocity(shooterSettings.velocity);
        shootTimer.start();
        isShooting = true;
      } else {
        // Not ready to shoot, move the elevator, wrist, and drivetrain
        elevatorSubsystem.moveToPosition(shooterSettings.height);
        wristSubsystem.moveToPosition(shooterSettings.angle);
      }
    }
    super.execute();
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
    super.end(interrupted);
  }

}
