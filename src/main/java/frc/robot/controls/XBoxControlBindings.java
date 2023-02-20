package frc.robot.controls;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.TeleopDriveConstants.XBOX_CONTROLLER_DEADBAND;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;

public class XBoxControlBindings implements ControlBindings {

  private final CommandXboxController driverController = new CommandXboxController(0);;

  @Override
  public Optional<Trigger> elevatorDown() {
    return Optional.of(driverController.x());
  }

  @Override
  public Optional<Trigger> elevatorUp() {
    return Optional.of(driverController.y());
  }

  @Override
  public Optional<Trigger> fieldHeadingDrive() {
    return Optional.of(driverController.povDown());
  }

  @Override
  public Optional<Trigger> fieldOrientedDrive() {
    return Optional.of(driverController.povUp());
  }

  @Override
  public Optional<Trigger> intakeCone() {
    return Optional.of(driverController.leftTrigger());
  }

  @Override
  public Optional<Trigger> intakeCube() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> reseedSteerMotors() {
    return Optional.of(driverController.start());
  }

  @Override
  public Optional<Trigger> resetPose() {
    return Optional.of(driverController.back());
  }

  @Override
  public Optional<Trigger> shootConeHigh() {
    return Optional.of(driverController.rightTrigger());
  }

  @Override
  public Optional<Trigger> shootConeLow() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> shootConeMid() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> shootCubeHigh() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> shootCubeLow() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> shootCubeMid() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> shooterIn() {
    return Optional.of(driverController.leftBumper());
  }

  @Override
  public Optional<Trigger> shooterOut() {
    return Optional.of(driverController.rightBumper());
  }

  @Override
  public Optional<Trigger> tuneShoot() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(driverController.povRight());
  }

  @Override
  public Optional<Trigger> wristDown() {
    return Optional.of(driverController.a());
  }

  @Override
  public Optional<Trigger> wristUp() {
    return Optional.of(driverController.x());
  }
  
  @Override
  public DoubleSupplier translationX() {
    return () ->-modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND;
  }
  @Override
  public DoubleSupplier translationY() {
    return () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND;
  }
  
  @Override
  public DoubleSupplier omega() {
    return () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2;
  }

  @Override
  public Supplier<Optional<Rotation2d>> heading() {
    return () -> {
      final var thetaX = -modifyAxis(driverController.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
      final var thetaY = -modifyAxis(driverController.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
      final var centered = MathUtil.applyDeadband(thetaX, XBOX_CONTROLLER_DEADBAND) == 0
          && MathUtil.applyDeadband(thetaY, XBOX_CONTROLLER_DEADBAND) == 0;
  
      if (centered) {
        // Hold heading when stick is centered
        return Optional.empty();
      }
        // Calculate heading from Y-Axis to X, Y coordinates
        return Optional.of(new Rotation2d(thetaX, thetaY));
    };
  }

  @Override
  public BooleanSupplier driverWantsControl() {
    return () -> modifyAxis(driverController.getLeftY()) != 0.0 
        || modifyAxis(driverController.getLeftX()) != 0.0
        || modifyAxis(driverController.getLeftY()) != 0.0
        || modifyAxis(driverController.getLeftX()) != 0.0;
  }
  
  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, XBOX_CONTROLLER_DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
