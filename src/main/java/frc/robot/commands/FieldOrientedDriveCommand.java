package frc.robot.commands;

import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.Y_RATE_LIMIT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command for teleop driving where translation is field oriented and rotation velocity is controlled by the driver.
 * 
 * Translation is specified on the field-relative coordinate system. The Y-axis runs parallel to the alliance wall, left
 * is positive. The X-axis runs down field toward the opposing alliance wall, away from the alliance wall is positive.
 */
public class FieldOrientedDriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Rotation2d> robotAngleSupplier;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(X_RATE_LIMIT);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(Y_RATE_LIMIT);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT);

  /**
   * Constructor
   * @param drivetrainSubsystem drivetrain
   * @param robotAngleSupplier supplier for the current angle of the robot
   * @param translationXSupplier supplier for translation X component, in meters per second
   * @param translationYSupplier supplier for translation Y component, in meters per second
   * @param rotationSupplier supplier for rotation component, in radians per second
   */
  public FieldOrientedDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Rotation2d> robotAngleSupplier,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngleSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    var robotAngle = robotAngleSupplier.get();

    // Calculate field relative speeds
    var chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
    var fieldSpeeds = 
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(robotAngle);
    var robotSpeeds = new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
    
    // Reset the slew rate limiters, in case the robot is already moving
    translateXRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
    translateYRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
    rotationRateLimiter.reset(robotSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translateXRateLimiter.calculate(translationXSupplier.getAsDouble()),
            translateYRateLimiter.calculate(translationYSupplier.getAsDouble()),
            rotationRateLimiter.calculate(rotationSupplier.getAsDouble()),
            robotAngleSupplier.get()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }
}
