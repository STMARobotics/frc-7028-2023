package frc.robot.controls;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBindings {
  Optional<Trigger> resetPose();
  Optional<Trigger> reseedSteerMotors();
  Optional<Trigger> fieldHeadingDrive();
  Optional<Trigger> fieldOrientedDrive();
  Optional<Trigger> wheelsToX();
  Optional<Trigger> elevatorUp();
  Optional<Trigger> elevatorDown();
  Optional<Trigger> wristUp();
  Optional<Trigger> wristDown();
  Optional<Trigger> shooterIn();
  Optional<Trigger> shooterOut();
  Optional<Trigger> manualIntake();
  Optional<Trigger> autoIntake();
  Optional<Trigger> doubleStationPickup();
  Optional<Trigger> shootAutomatically();
  Optional<Trigger> shootHigh();
  Optional<Trigger> shootMid();
  Optional<Trigger> shootLow();
  Optional<Trigger> launchCube();
  Optional<Trigger> driveAndPickup();
  DoubleSupplier translationX();
  DoubleSupplier translationY();
  DoubleSupplier omega();
  Supplier<Optional<Rotation2d>> heading();
  BooleanSupplier driverWantsControl();
}
