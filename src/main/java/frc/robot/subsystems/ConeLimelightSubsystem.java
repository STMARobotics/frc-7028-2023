package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

/**
 * Extends LimelightSubsystem to add information about the retroreflective target distance.
 */
public class ConeLimelightSubsystem extends LimelightSubsystem {
  public final LimelightConfig limelightConfig;
  private Profile activeProfile = Profile.TOP;

  public ConeLimelightSubsystem(LimelightConfig limelightConfig) {
    super(limelightConfig.getNetworkTableName());
    this.limelightConfig = limelightConfig;
  }

  /**
   * Gets the distance from the Limelight to the target. If no target is acquired this will return zero. This value is
   * the distance straight ahead from the limelight to the target, (parallel to the floor.) Use
   * {@link #getDistanceToTarget()} to get the distance from the robot to the target.
   * 
   * @return distance from the Limelight to the target
   */
  private double getLimelightDistanceToTarget() {
    var results = getLatestResults();
    if (results.valid && results.targets_Retro.length > 0) {
      return (activeProfile.targetHeight - limelightConfig.getMountHeight())
          / Math.tan(Units.degreesToRadians(limelightConfig.getMountAngle() + results.targets_Retro[0].tx));
    }
    return 0.0;
  }

  /**
  * Gets the distance from the center of the robot front bumper to the base of the target. If no target is acquired this
  * will return zero. This method accounts for the limelight being mounted off-center and behind the front bumper.
  * 
  * @return distance from the center of the robot front bumper to the base of the target
  */
  public double getDistanceToTarget() {
    return Math.sqrt(Math.pow(getLimelightDistanceToTarget(), 2)
        + Math.pow(limelightConfig.getMountDistanceFromCenter(), 2)) - limelightConfig.getMountDepth();
  }

  /**
   * Sets the active profile. This controls the pipeline and target info
   * @param profile profile
   */
  public void setActiveProfile(Profile profile) {
    setPipelineId(profile.pipelineId);
  }

}