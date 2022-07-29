package frc.robot.swerve;

/**
 * A swerve module configuration.
 * <p>
 * A configuration represents a unique mechanical configuration of a module. For example, the Swerve Drive Specialties
 * Mk3 swerve module has two configurations, standard and fast, and therefore should have two configurations
 * ({@link SdsModuleConfigurations#MK3_STANDARD} and {@link SdsModuleConfigurations#MK3_FAST} respectively).
 */
public enum ModuleConfiguration {

  MK4_L1(
    0.10033,
    (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
    true,
    (15.0 / 32.0) * (10.0 / 60.0),
    true),

MK4_L2(
    0.10033,
    (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
    true,
    (15.0 / 32.0) * (10.0 / 60.0),
    true),

MK4_L3(
    0.10033,
    (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
    true,
    (15.0 / 32.0) * (10.0 / 60.0),
    true),
    
MK4_L4(
    0.10033,
    (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
    true,
    (15.0 / 32.0) * (10.0 / 60.0),
    true);

  private final double wheelDiameter;
  private final double driveReduction;
  private final boolean driveInverted;

  private final double steerReduction;
  private final boolean steerInverted;

  /**
   * Creates a new module configuration.
   *
   * @param wheelDiameter  The diameter of the module's wheel in meters.
   * @param driveReduction The overall drive reduction of the module. Multiplying
   *                       motor rotations by this value
   *                       should result in wheel rotations.
   * @param driveInverted  Whether the drive motor should be inverted. If there is
   *                       an odd number of gea reductions
   *                       this is typically true.
   * @param steerReduction The overall steer reduction of the module. Multiplying
   *                       motor rotations by this value
   *                       should result in rotations of the steering pulley.
   * @param steerInverted  Whether the steer motor should be inverted. If there is
   *                       an odd number of gear reductions
   *                       this is typically true.
   */
  private ModuleConfiguration(double wheelDiameter, double driveReduction, boolean driveInverted,
      double steerReduction, boolean steerInverted) {
    this.wheelDiameter = wheelDiameter;
    this.driveReduction = driveReduction;
    this.driveInverted = driveInverted;
    this.steerReduction = steerReduction;
    this.steerInverted = steerInverted;
  }

  /**
   * Gets the diameter of the wheel in meters.
   */
  public double getWheelDiameter() {
    return wheelDiameter;
  }

  /**
   * Gets the overall reduction of the drive system.
   * <p>
   * If this value is multiplied by drive motor rotations the result would be
   * drive wheel rotations.
   */
  public double getDriveReduction() {
    return driveReduction;
  }

  /**
   * Gets if the drive motor should be inverted.
   */
  public boolean isDriveInverted() {
    return driveInverted;
  }

  /**
   * Gets the overall reduction of the steer system.
   * <p>
   * If this value is multiplied by steering motor rotations the result would be
   * steering pulley rotations.
   */
  public double getSteerReduction() {
    return steerReduction;
  }

  /**
   * Gets if the steering motor should be inverted.
   */
  public boolean isSteerInverted() {
    return steerInverted;
  }

  @Override
  public String toString() {
    return "ModuleConfiguration{" +
        "wheelDiameter=" + wheelDiameter +
        ", driveReduction=" + driveReduction +
        ", driveInverted=" + driveInverted +
        ", steerReduction=" + steerReduction +
        ", steerInverted=" + steerInverted +
        '}';
  }
}
