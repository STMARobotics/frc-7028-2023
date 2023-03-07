package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PickupConstants;
import frc.robot.commands.autonomous.TransitCommand;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Command to pick up a game piece from the double station - and hold elevator and wrist until interrupted */
public class DoubleStationCommand extends CommandBase {

  private static final double SPEED_COEFFICIENT = 0.2;
  private static final double XY_SLEW_RATE = 2.0;
  private static final double THETA_TOLERANCE = .01;

  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2);
  
  private final ProfiledPIDController thetaController = new ProfiledPIDController(4.0, 0.0, 0, OMEGA_CONSTRAINTS);

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<Pose2d> poseSupplier;
  private final LimelightSubsystem limelightSubsystem;
  private final LimelightProfile profile;
  private final String className;
  private final LimelightCalcs limelightCalcs;

  private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(XY_SLEW_RATE);
  private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(XY_SLEW_RATE);

  private Rotation2d lastTargetHeading;

  /**
   * 
   * @param elevatorMeters
   * @param wristRadians
   * @param intakeDutyCycle
   * @param xSupplier x speed - this will be limited
   * @param ySupplier y speed - this will be limited
   * @param elevatorSubsystem
   * @param wristSubsystem
   * @param drivetrainSubsystem
   * @param shooterSubsystem
   * @param poseSupplier
   * @param limelightSubsystem
   * @param profile
   * @param className
   */
  public DoubleStationCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem, DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem,
      Supplier<Pose2d> poseSupplier, LimelightSubsystem limelightSubsystem, LimelightProfile profile,
      String className) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.poseSupplier = poseSupplier;
    this.limelightSubsystem = limelightSubsystem;
    this.profile = profile;
    this.className = className;

    this.limelightCalcs = new LimelightCalcs(profile.cameraToRobot, profile.targetHeight,
        profile.cameraOnElevator ? elevatorSubsystem::getElevatorTopPosition : () -> 0.0);

    thetaController.setTolerance(THETA_TOLERANCE);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(profile.pipelineId);
    elevatorSubsystem.moveToPosition(PickupConstants.DOUBLE_ELEVATOR_HEIGHT);
    wristSubsystem.moveToPosition(PickupConstants.DOUBLE_WRIST_ANGLE);
    shooterSubsystem.shootDutyCycle(PickupConstants.DOUBLE_INTAKE_DUTY_CYCLE);
    var chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
    xRateLimiter.reset(chassisSpeeds.vxMetersPerSecond);
    yRateLimiter.reset(chassisSpeeds.vyMetersPerSecond);
    lastTargetHeading = null;
    thetaController.reset(poseSupplier.get().getRotation().getRadians(), chassisSpeeds.omegaRadiansPerSecond);
  }
  
  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(PickupConstants.DOUBLE_ELEVATOR_HEIGHT);

    if (shooterSubsystem.hasCone() || shooterSubsystem.hasCube()) {
      shooterSubsystem.stop();
      wristSubsystem.parkWrist();
    } else {
      shooterSubsystem.shootDutyCycle(PickupConstants.DOUBLE_INTAKE_DUTY_CYCLE);
      wristSubsystem.moveToPosition(PickupConstants.DOUBLE_WRIST_ANGLE);
    }

    var xSpeed = xRateLimiter.calculate(xSupplier.getAsDouble() * SPEED_COEFFICIENT);
    var ySpeed = yRateLimiter.calculate(ySupplier.getAsDouble() * SPEED_COEFFICIENT);

    var drivetrainHeading = poseSupplier.get().getRotation();
    var detectorTarget = limelightSubsystem.getLatestDetectorTarget();
    if (detectorTarget.isPresent()) {
      var target = detectorTarget.get();
      if (className.equalsIgnoreCase(target.className)) {
        var targetInfo = limelightCalcs.getRobotRelativeTargetInfo(detectorTarget.get());
        lastTargetHeading = drivetrainHeading.plus(targetInfo.angle);
        thetaController.setGoal(lastTargetHeading.getRadians());
      }
    }

    double omegaSpeed = 0.0;
    if (lastTargetHeading != null && !thetaController.atGoal()) {
      omegaSpeed = thetaController.calculate(drivetrainHeading.getRadians());
    }
    drivetrainSubsystem.drive(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.activeStop();
    wristSubsystem.parkWrist();
    drivetrainSubsystem.stop();
    var transitCommand = new TransitCommand(elevatorSubsystem, wristSubsystem, shooterSubsystem);
    transitCommand.addRequirements(drivetrainSubsystem);
    transitCommand.schedule();
  }

}
