package frc.robot.commands;

import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;
import static frc.robot.Constants.TeleopDriveConstants.HEADING_MAX_ACCELERATION;
import static frc.robot.Constants.TeleopDriveConstants.HEADING_MAX_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.HEADING_TOLERANCE;
import static frc.robot.Constants.TeleopDriveConstants.HEADING_kD;
import static frc.robot.Constants.TeleopDriveConstants.HEADING_kI;
import static frc.robot.Constants.TeleopDriveConstants.HEADING_kP;
import static frc.robot.Constants.TeleopDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.Y_RATE_LIMIT;
import static java.lang.Math.PI;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command for teleop driving where translation and heading are field oriented.
 * 
 * Translation is specified on the field-relative coordinate system. The Y-axis runs parallel to the alliance wall, left
 * is positive. The X-axis runs down field toward the opposing alliance wall, away from the alliance wall is positive.
 * 
 * Rotation is specified as a point to make it easy to use with a joystick. The heading is the angle from the X-axis
 * to the point.
 */
public class FieldHeadingDriveCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Rotation2d> robotAngleSupplier;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaXSupplier;
  private final DoubleSupplier thetaYSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(X_RATE_LIMIT);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(Y_RATE_LIMIT);

  private final ProfiledPIDController thetaController;

  /**
   * Constructor.
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param robotAngleSupplier supplier for the robot's current heading
   * @param translationXSupplier supplier for field-oriented X velocity in meters/sec
   * @param translationYSupplier supplier for field-oriented Y velocity in meters/sec
   * @param thetaXSupplier X supplier for heading
   * @param thetaYSupplier Y supplier for heading
   */
  public FieldHeadingDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Rotation2d> robotAngleSupplier,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier thetaXSupplier,
      DoubleSupplier thetaYSupplier) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngleSupplier;
    this.xSupplier = translationXSupplier;
    this.ySupplier = translationYSupplier;
    this.thetaXSupplier = thetaXSupplier;
    this.thetaYSupplier = thetaYSupplier;

    addRequirements(drivetrainSubsystem);

    TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(HEADING_MAX_VELOCITY, HEADING_MAX_ACCELERATION);
        
    thetaController = new ProfiledPIDController(HEADING_kP, HEADING_kI, HEADING_kD, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-PI, PI);
    thetaController.setTolerance(Units.degreesToRadians(HEADING_TOLERANCE));
  }

  @Override
  public void initialize() {
    var robotAngle = robotAngleSupplier.get();

    // Reset the theta controller to the current heading
    thetaController.reset(robotAngle.getRadians());

    // Calculate field relative speeds by rotating the chasis relative speeds by the robot's angle on the field
    var chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
    var fieldSpeeds = 
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(robotAngle);
    var robotSpeeds = new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);

    // Reset the slew rate limiters, in case the robot is already moving
    translateXRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
    translateYRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
  }

  @Override
  public void execute() {
    final var thetaX = thetaXSupplier.getAsDouble();
    final var thetaY = thetaYSupplier.getAsDouble();
    final var centered = MathUtil.applyDeadband(thetaX, DEADBAND) == 0 && MathUtil.applyDeadband(thetaY, DEADBAND) == 0;

    Rotation2d heading;
    if (centered) {
      // Hold heading when stick is centered
      heading = robotAngleSupplier.get();
    } else {
      // Calculate heading from Y-Axis to X, Y coordinates
      heading = new Rotation2d(thetaX, thetaY);
    }

    // Calculate the angular rate for the robot to turn
    var omega = thetaController.calculate(robotAngleSupplier.get().getRadians(), heading.getRadians());
    if (thetaController.atGoal() || centered) {
      omega = 0;
    }

    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        translateXRateLimiter.calculate(xSupplier.getAsDouble()),
        translateYRateLimiter.calculate(ySupplier.getAsDouble()),
        omega,
        robotAngleSupplier.get()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }
  
}
