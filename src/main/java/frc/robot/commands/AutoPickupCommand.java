package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
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

  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2.0,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
  
  private final ProfiledPIDController thetaController = new ProfiledPIDController(0.8, 0.0, 0, OMEGA_CONSTRAINTS);

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

  private Rotation2d lastTargetHeading;

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
        profile.cameraOnElevator ? elevatorSubsystem::getElevatorPosition : () -> 0.0;
    limelightCalcs = new LimelightCalcs(profile.cameraToRobot, profile.targetHeight, cameraHeightOffset);

    addRequirements(elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(profile.pipelineId);
    thetaController.reset(robotPoseSupplier.get().getRotation().getRadians());
  }
  
  @Override
  public void execute() {
    var drivetrainHeading = robotPoseSupplier.get().getRotation();
    var detectorTarget = limelightSubsystem.getLatestDetectorTarget();
    if (detectorTarget.isPresent()) {
      var targetInfo = limelightCalcs.getRobotRelativeTargetInfo(detectorTarget.get());
      lastTargetHeading = drivetrainHeading.plus(targetInfo.angle);
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

      var xySpeed = new Translation2d(forwardSpeed, 0).rotateBy(lastTargetHeading.minus(drivetrainHeading));
      drivetrainSubsystem.drive(new ChassisSpeeds(xySpeed.getX(), xySpeed.getY(), omegaSpeed));
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
