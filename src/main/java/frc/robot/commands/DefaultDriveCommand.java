package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(5 * Math.PI);

  /**
   * Constructor
   * @param drivetrainSubsystem drivetrain
   * @param translationXSupplier supplier for translation X component, in meters per second
   * @param translationYSupplier supplier for translation Y component, in meters per second
   * @param rotationSupplier supplier for rotation component, in radians per second
   */
  public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translateXRateLimiter.calculate(translationXSupplier.getAsDouble()),
            translateYRateLimiter.calculate(translationYSupplier.getAsDouble()),
            rotationRateLimiter.calculate(rotationSupplier.getAsDouble()),
            drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
