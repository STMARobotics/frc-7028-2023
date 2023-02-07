package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Holds configuration for a limelight to use with ShooterLimelightSubsystem.
 */
public class LimelightConfig {

  private final String networkTableName;

  private final Transform3d cameraToRobot;
  
  public LimelightConfig(String networkTableName, Transform3d cameraToRobot) {
    this.networkTableName = networkTableName;
    this.cameraToRobot = cameraToRobot;
  }

  /**
   * Gets the name of the Limelight network table
   * @return network table name
   */
  public String getNetworkTableName() {
    return networkTableName;
  }

  /**
   * Gets the physical location of the camera on the robot, relative to the center of the robot.
   * @return
   */
  public Transform3d getCameraToRobot() {
    return cameraToRobot;
  }

}