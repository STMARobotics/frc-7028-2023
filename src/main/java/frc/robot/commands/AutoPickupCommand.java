package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoPickupCommand extends CommandBase {

  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.035;
  private static final double THETA_TOLERANCE = .02;

  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 3.0,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 2);
  
  private final ProfiledPIDController thetaController = new ProfiledPIDController(3.0, 0.0, 0, OMEGA_CONSTRAINTS);

  private final double elevatorMeters;
  private final double wristRadians;
  private final double intakeDutyCycle;
  private final double forwardSpeed;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final Profile profile;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final BooleanSupplier finishedSuppiler;

  private final LimelightCalcs limelightCalcs;
  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0);

  private Rotation2d lastTargetHeading;
  private Double lastTargetDistance;

  public AutoPickupCommand(
      double elevatorMeters, double wristRadians, double intakeDutyCycle, double forwardSpeed, 
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, DrivetrainSubsystem drivetrainSubsystem, 
      ShooterSubsystem shooterSubsystem, Supplier<Pose2d> poseSupplier, LimelightSubsystem limelightSubsystem,
      Profile profile, BooleanSupplier finishedSuppiler) {

    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.intakeDutyCycle = intakeDutyCycle;
    this.forwardSpeed = forwardSpeed;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.robotPoseSupplier = poseSupplier;
    this.limelightSubsystem = limelightSubsystem;
    this.profile = profile;
    this.finishedSuppiler = finishedSuppiler;

    DoubleSupplier cameraHeightOffset = 
        profile.cameraOnElevator ? elevatorSubsystem::getElevatorTopPosition : () -> 0.0;
    limelightCalcs = new LimelightCalcs(profile.cameraToRobot, profile.targetHeight, cameraHeightOffset);
    thetaController.setTolerance(THETA_TOLERANCE);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(profile.pipelineId);
    var chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
    var robotAngle = robotPoseSupplier.get().getRotation();
    thetaController.reset(robotPoseSupplier.get().getRotation().getRadians(), chassisSpeeds.omegaRadiansPerSecond);
    lastTargetDistance = null;
    lastTargetHeading = null;

    var fieldSpeeds = 
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(robotAngle);
    var robotSpeeds = new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
    
    // Reset the slew rate limiters, in case the robot is already moving
    xSlewRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
    ySlewRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
  }
  
  @Override
  public void execute() {
    var drivetrainHeading = robotPoseSupplier.get().getRotation();
    var detectorTarget = limelightSubsystem.getLatestDetectorTarget();
    if (detectorTarget.isPresent()) {
      var targetInfo = limelightCalcs.getRobotRelativeTargetInfo(detectorTarget.get());
      lastTargetHeading = drivetrainHeading.plus(targetInfo.angle);
      lastTargetDistance = targetInfo.distance;
      thetaController.setGoal(lastTargetHeading.getRadians());
    }

    if (lastTargetHeading == null) {
      // We've never seen a game piece, just drive robot foward
      drivetrainSubsystem.drive(new ChassisSpeeds(forwardSpeed, 0, 0));
    } else {
      var omegaSpeed = thetaController.calculate(drivetrainHeading.getRadians());
      if (thetaController.atGoal()) {
        omegaSpeed = 0;
      }

      var chaseSpeed = forwardSpeed;
      if (lastTargetDistance > .8) {
        chaseSpeed = 0.8;
      }
      var xySpeed = new Translation2d(chaseSpeed, 0).rotateBy(lastTargetHeading.minus(drivetrainHeading));
      chaseSpeed = xSlewRateLimiter.calculate(chaseSpeed);
      drivetrainSubsystem.drive(new ChassisSpeeds(
          xSlewRateLimiter.calculate(xySpeed.getX()),
          ySlewRateLimiter.calculate(xySpeed.getY()),
          omegaSpeed));
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
    return finishedSuppiler.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

}
