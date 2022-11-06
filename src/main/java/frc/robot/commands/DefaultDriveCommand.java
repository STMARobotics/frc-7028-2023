package frc.robot.commands;

import static frc.robot.Constants.ArcadeDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.ArcadeDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.ArcadeDriveConstants.Y_RATE_LIMIT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
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
   * @param robotAngleSuppliser supplier for the current angle of the robot
   * @param translationXSupplier supplier for translation X component, in meters per second
   * @param translationYSupplier supplier for translation Y component, in meters per second
   * @param rotationSupplier supplier for rotation component, in radians per second
   */
  public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
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
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
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
