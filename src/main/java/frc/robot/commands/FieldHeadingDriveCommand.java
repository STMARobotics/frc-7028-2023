package frc.robot.commands;

import static frc.robot.Constants.ArcadeDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.ArcadeDriveConstants.Y_RATE_LIMIT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FieldHeadingDriveCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Rotation2d> robotAngleSupplier;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaXSupplier;
  private final DoubleSupplier omegaYSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(X_RATE_LIMIT);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(Y_RATE_LIMIT);

  private final ProfiledPIDController thetaController;

  public FieldHeadingDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Rotation2d> robotAngle,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier omegaXSupplier,
      DoubleSupplier omegaYSupplier) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngle;
    this.xSupplier = translationXSupplier;
    this.ySupplier = translationYSupplier;
    this.omegaXSupplier = omegaXSupplier;
    this.omegaYSupplier = omegaYSupplier;

    addRequirements(drivetrainSubsystem);

    TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2);
        
    thetaController = new ProfiledPIDController(2, 0, 0.0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(1.5));

  }

  @Override
  public void initialize() {
    thetaController.reset(robotAngleSupplier.get().getRadians());
    // TODO reset slew limiters to current speed
  }

  @Override
  public void execute() {
    final var omegaX = omegaXSupplier.getAsDouble();
    final var omegaY = omegaYSupplier.getAsDouble();
    final var centered = 
        MathUtil.applyDeadband(omegaX, 0.1) == 0 && MathUtil.applyDeadband(omegaY, 0.1) == 0;

    Rotation2d heading;
    if (centered) {
      // Hold heading when stick is centered
      heading = robotAngleSupplier.get();
    } else {
      // Calculate heading from Y-Axis to X, Y coordinates
      heading = new Rotation2d(omegaX, omegaY);
    }
    SmartDashboard.putNumber("heading", heading.getDegrees());

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
