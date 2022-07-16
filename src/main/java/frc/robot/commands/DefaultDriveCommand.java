package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  // FIXME these limits are a guess
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
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    m_drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translateXRateLimiter.calculate(m_translationXSupplier.getAsDouble()),
            translateYRateLimiter.calculate(m_translationYSupplier.getAsDouble()),
            rotationRateLimiter.calculate(m_rotationSupplier.getAsDouble()),
            m_drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
