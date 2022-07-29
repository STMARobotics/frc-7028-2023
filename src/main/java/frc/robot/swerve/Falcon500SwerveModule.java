package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Falcon500SwerveModule {
  private final Falcon500DriveController driveController;
  private final Falcon500SteerController steerController;

  public Falcon500SwerveModule(Falcon500DriveController driveController, Falcon500SteerController steerController) {
    this.driveController = driveController;
    this.steerController = steerController;
  }

  public double getDriveVelocity() {
    return driveController.getStateVelocity();
  }

  public double getSteerAngle() {
    return steerController.getStateAngle();
  }

  public void set(double velocity, double steerAngle) {
    steerAngle %= (2.0 * Math.PI);
    if (steerAngle < 0.0) {
      steerAngle += 2.0 * Math.PI;
    }

    double difference = steerAngle - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead
    // of [0, 2pi)
    if (difference >= Math.PI) {
      steerAngle -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      steerAngle += 2.0 * Math.PI;
    }
    difference = steerAngle - getSteerAngle(); // Recalculate difference

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      steerAngle += Math.PI;
      velocity *= -1.0;
    }

    // Put the target angle back into the range [0, 2pi)
    steerAngle %= (2.0 * Math.PI);
    if (steerAngle < 0.0) {
      steerAngle += 2.0 * Math.PI;
    }

    driveController.setReferenceVelocity(velocity);
    steerController.setReferenceAngle(steerAngle);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
  }

  /**
   * Sets the neutral mode for the drive and steer motors
   * @param neutralMode neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    steerController.setNeutralMode(neutralMode);
    driveController.setNeutralMode(neutralMode);
  }

}