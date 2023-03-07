package frc.robot.controls;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.TeleopDriveConstants.JOYSTICK_DEADBAND;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;

public class JoystickControlBindings implements ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  @Override
  public Optional<Trigger> elevatorDown() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> elevatorUp() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> fieldHeadingDrive() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> fieldOrientedDrive() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> manualIntake() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> autoIntake() {
    return Optional.of(leftJoystick.trigger());
  }
  
  @Override
  public Optional<Trigger> doubleStationPickup() {
    return Optional.of(leftJoystick.button(2));
  }

  @Override
  public Optional<Trigger> reseedSteerMotors() {
    return Optional.of(leftJoystick.button(6));
  }

  @Override
  public Optional<Trigger> resetPose() {
    return Optional.of(leftJoystick.povUp());
  }
  
  @Override
  public Optional<Trigger> shootAutomatically() {
    return Optional.of(rightJoystick.trigger());
  }

  @Override
  public Optional<Trigger> shootHigh() {
    return Optional.of(rightJoystick.povUp());
  }

  @Override
  public Optional<Trigger> shootLow() {
    return Optional.of(rightJoystick.povDown());
  }

  @Override
  public Optional<Trigger> shootMid() {
    return Optional.of(rightJoystick.povLeft());
  }

  @Override
  public Optional<Trigger> shooterIn() {
    return Optional.of(rightJoystick.button(3));
  }

  @Override
  public Optional<Trigger> shooterOut() {
    return Optional.of(rightJoystick.button(2));
  }

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(leftJoystick.button(4));
  }

  @Override
  public Optional<Trigger> wristDown() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> wristUp() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> launchCube() {
    return Optional.of(rightJoystick.button(8));
  }

  @Override
  public DoubleSupplier translationX() {
    return () -> -modifyAxis(leftJoystick.getY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
  }
  @Override
  public DoubleSupplier translationY() {
    return () -> -modifyAxis(leftJoystick.getX()) * MAX_VELOCITY_METERS_PER_SECOND;
  }
  
  @Override
  public DoubleSupplier omega() {
    return () -> -modifyAxis(rightJoystick.getX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2;
  }

  @Override
  public Supplier<Optional<Rotation2d>> heading() {
    return () -> {
      final var thetaX = -modifyAxis(rightJoystick.getY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
      final var thetaY = -modifyAxis(rightJoystick.getX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
      final var centered = MathUtil.applyDeadband(thetaX, JOYSTICK_DEADBAND) == 0 && MathUtil.applyDeadband(thetaY, JOYSTICK_DEADBAND) == 0;
  
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
    return () -> modifyAxis(leftJoystick.getY()) != 0.0 
        || modifyAxis(leftJoystick.getX()) != 0.0
        || modifyAxis(rightJoystick.getY()) != 0.0
        || modifyAxis(rightJoystick.getX()) != 0.0;
  }
  
  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
