package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FieldHeadingDriveCommand extends CommandBase {

  // Rotation of -90-degrees to get from positive X-axis (right) to the positive Y-axis (up)
  private static final Rotation2d X_TO_Y = new Rotation2d(-Math.PI / 2);

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaXSupplier;
  private final DoubleSupplier omegaYSupplier;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  private final ProfiledPIDController thetaController;

  public FieldHeadingDriveCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaXSupplier,
      DoubleSupplier omegaYSupplier,
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Pose2d> poseSupplier) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaXSupplier = omegaXSupplier;
    this.omegaYSupplier = omegaYSupplier;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseSupplier = poseSupplier;

    // FIXME tune theta constraints
    TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(Math.PI * 4, 2 * Math.PI);
    
    // FIXME set theta PID values
    thetaController = new ProfiledPIDController(5, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(3));

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    thetaController.reset(poseSupplier.get().getRotation().getRadians());
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
      heading = poseSupplier.get().getRotation();
    } else {
      // Calculate heading from Y-Axis to X, Y coordinates
      heading = new Rotation2d(omegaX, omegaY).rotateBy(X_TO_Y);
    }
    SmartDashboard.putNumber("Heading", heading.getDegrees());

    // Calculate the angular rate for the robot to turn
    var omega = thetaController.calculate(poseSupplier.get().getRotation().getRadians(), heading.getRadians());
    if (thetaController.atGoal() || centered) {
      omega = 0;
    }

    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        xSupplier.getAsDouble(), ySupplier.getAsDouble(), omega, drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }
  
}
