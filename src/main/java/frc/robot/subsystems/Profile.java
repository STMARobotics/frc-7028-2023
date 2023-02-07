package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ConeShootingConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.VelocityAngleInterpolator;

/**
 * Shooter profiles enum
 */
public enum Profile {

  /** Middle of the Top cone vision target */
  TOP(1, Units.inchesToMeters(24.125), ConeShootingConstants.TOP_TABLE, VisionConstants.SHOOTER_CAMERA_TO_ROBOT),

  /** Middle of the Middle cone vision target */
  MIDDLE(0, Units.inchesToMeters(43.875), ConeShootingConstants.MIDDLE_TABLE, VisionConstants.SHOOTER_CAMERA_TO_ROBOT);

  /** ID for the Limelight profile */
  public final int pipelineId;

  /** Height of the target in meters */
  public final double targetHeight;

  /** Lookup table for shooter velocity, angle, and height */
  public final VelocityAngleInterpolator lookupTable;

  /** Camera to robot transform for the targeting camera */
  public final Transform3d cameraToRobot;

  /**
   * Create a limelight profile
   * @param pipelineId pipeline ID
   * @param targetHeight
   */
  private Profile(
      int pipelineId, double targetHeight, VelocityAngleInterpolator lookupTable,  Transform3d cameraToRobot) {
    this.pipelineId = pipelineId;
    this.targetHeight = targetHeight;
    this.lookupTable = lookupTable;
    this.cameraToRobot = cameraToRobot;
  }

}