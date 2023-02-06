package frc.robot.subsystems;

/**
 * Holds configuration for a limelight to use with ShooterLimelightSubsystem.
 */
public class LimelightConfig {

  private String networkTableName;

  private double mountHeight;

  private double mountDepth;

  private double mountAngle;

  private double mountDistanceFromCenter;

  /**
   * Gets the name of the Limelight network table
   * @return network table name
   */
  public String getNetworkTableName() {
    return networkTableName;
  }

  /**
   * Gets the height of the limelight on the bot in meters
   * @return limelight height
   */
  public double getMountHeight() {
    return mountHeight;
  }

  /**
   * Distance Limelight is mounted from the front frame of the bot
   * @return mount depth
   */
  public double getMountDepth() {
    return mountDepth;
  }

  /**
   * Angle of the limelight in degrees
   * @return mount angle
   */
  public double getMountAngle() {
    return mountAngle;
  }

  /**
   * Distance Limelight is mounted from the centerline of the bot
   * @return limelight distance from center
   */
  public double getMountDistanceFromCenter() {
    return mountDistanceFromCenter;
  }

  /**
   * Builder to create immutable LimelightConfig.
   */
  public static class Builder {
    
    private LimelightConfig limelightConfig = new LimelightConfig();

    /** Creates an instance of the Builder */
    public static Builder create() {
      return new Builder();
    }

    /**
     * Name of the Limelight network table
     * @param networkTableName network table name
     */
    public Builder withNetworkTableName(String networkTableName) {
      limelightConfig.networkTableName = networkTableName;
      return this;
    }

    /**
     * Height of the limelight on the bot in meters
     * @param mountingHeight mounting height
     */
    public Builder withMountingHeight(double mountingHeight) {
      limelightConfig.mountHeight = mountingHeight;
      return this;
    }

    /**
     * Distance Limelight is mounted from the front frame of the bot
     * @param mountDepth mount depth
     */
    public Builder withMountDepth(double mountDepth) {
      limelightConfig.mountDepth = mountDepth;
      return this;
    }

    /**
     * Angle of the limelight in degrees
     * @param mountingAngle mount angle
     */
    public Builder withMountingAngle(double mountingAngle) {
      limelightConfig.mountAngle = mountingAngle;
      return this;
    }

    /**
     * Distance Limelight is mounted from the centerline of the bot
     * @param mountDistanceFromCenter distance from center
     */
    public Builder withMountDistanceFromCenter(double mountDistanceFromCenter) {
      limelightConfig.mountDistanceFromCenter = mountDistanceFromCenter;
      return this;
    }

    /**
     * Builds the Limelight config after setting the parameters
     * @return LimelightConfig instance
     */
    public LimelightConfig build() {
      return limelightConfig;
    }

  }

}