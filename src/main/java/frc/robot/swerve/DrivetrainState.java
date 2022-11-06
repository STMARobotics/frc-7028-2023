package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Class to hold the current drivetrain state. The state is the position, angle, and velocity of each module.
 */
public class DrivetrainState {
  private final SwerveModuleState[] swerveModuleStates;
  private final SwerveModulePosition[] swerveModulePositions;

  /**
   * Reads the module states to construct a new object.
   * @param modules modules in order. The order of the modules will match the order of the array
   * returned by {@link #getSwerveModulePositions()} and {@link #getSwerveModuleStates()}
   */
  public DrivetrainState(Falcon500SwerveModule... modules) {
    this.swerveModuleStates = new SwerveModuleState[modules.length];
    this.swerveModulePositions = new SwerveModulePosition[modules.length];
    for(int i = 0; i < modules.length; i++) {
      var angle = new Rotation2d(modules[i].getSteerAngle());
      swerveModuleStates[i] = new SwerveModuleState(modules[i].getDriveVelocity(), angle);
      swerveModulePositions[i] = new SwerveModulePosition(modules[i].getDrivePosition(), angle);
    }
  }

  /**
   * Gets the drivetrain states.
   * @return drivetrain states
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    return swerveModuleStates;
  }

  /**
   * Gets the drivetrain positions.
   * @return drivetrain positions
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return swerveModulePositions;
  }
}
