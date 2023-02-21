package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.limelight.VisionTargetInfo;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoConePickupCommand extends DriveToPoseCommand {

  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double DISTANCE_GOAL = -0.2;

  private static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_VELOCITY_METERS_PER_SECOND / 5.0,
      MAX_VELOCITY_METERS_PER_SECOND);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2.0,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

  private final double elevatorMeters;
  private final double wristRadians;
  private final double intakeDutyCycle;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final Profile profile;
  private final Supplier<Pose2d> robotPoseSupplier;

  private final LimelightCalcs limelightCalcs;

  private VisionTargetInfo lastTargetInfo = null;

  public AutoConePickupCommand(
      double elevatorMeters, double wristRadians, double intakeDutyCycle, double forwardSpeed, 
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, DrivetrainSubsystem drivetrainSubsystem, 
      ShooterSubsystem shooterSubsystem, Supplier<Pose2d> poseSupplier, LimelightSubsystem limelightSubsystem,
      Profile profile) {

    super(drivetrainSubsystem, poseSupplier, null, false, XY_CONSTRAINTS, OMEGA_CONSTRAINTS);

    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.intakeDutyCycle = intakeDutyCycle;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.robotPoseSupplier = poseSupplier;
    this.limelightSubsystem = limelightSubsystem;
    this.profile = profile;

    DoubleSupplier cameraHeightOffset = 
        profile.cameraOnElevator ? elevatorSubsystem::getElevatorPosition : () -> 0.0;
    limelightCalcs = new LimelightCalcs(profile.cameraToRobot, profile.targetHeight, cameraHeightOffset);

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(profile.pipelineId);
  }
  
  @Override
  public void execute() {
    var detectorTarget = limelightSubsystem.getLatestDetectorTarget();
    if (detectorTarget.isPresent()) {
      lastTargetInfo = limelightCalcs.getRobotRelativeTargetInfo(detectorTarget.get());
    }

    if (lastTargetInfo == null) {
      // We've never seen a game piece
      setGoalPose(null);
    } else {
      // Get the robot heading, and the robot-relative heading of the target
      var drivetrainHeading = robotPoseSupplier.get().getRotation();
      var targetHeading = drivetrainHeading.plus(lastTargetInfo.angle);
      var goalPose =
          new Pose2d(new Translation2d(lastTargetInfo.distance - DISTANCE_GOAL, targetHeading), targetHeading);
      setGoalPose(goalPose);
    }

    wristSubsystem.moveToPosition(wristRadians);
    elevatorSubsystem.moveToPosition(elevatorMeters);

    var readyToIntake =
        Math.abs(elevatorSubsystem.getElevatorPosition() - elevatorMeters) < ELEVATOR_TOLERANCE
            && Math.abs(wristSubsystem.getWristPosition() - wristRadians) < WRIST_TOLERANCE;

    if (readyToIntake) {
      shooterSubsystem.shootDutyCycle(intakeDutyCycle);
    }
    
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.hasCone();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

}
