package frc.robot.subsystems;

import static frc.robot.Constants.ConeShootingConstants.MIDDLE_TABLE;
import static frc.robot.Constants.ConeShootingConstants.TOP_TABLE;
import static frc.robot.Constants.VisionConstants.HIGH_LIMELIGHT_TO_ROBOT;
import static frc.robot.Constants.VisionConstants.LOW_LIMELIGHT_TO_ROBOT;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.math.VelocityAngleInterpolator;

/**
 * Shooter profiles enum
 */
public enum Profile {

  /** Middle of the Top cone vision target */
  SCORE_CONE_TOP(0, Units.inchesToMeters(44.0), TOP_TABLE, LOW_LIMELIGHT_TO_ROBOT, false),

  /** Middle of the Middle cone vision target */
  SCORE_CONE_MIDDLE(0, Units.inchesToMeters(24.125), MIDDLE_TABLE, HIGH_LIMELIGHT_TO_ROBOT, true),

  /** Gamepiece on the floor */
  PICKUP_GAMEPIECE_FLOOR(1, 0.0, null, HIGH_LIMELIGHT_TO_ROBOT, true);

  /** ID for the Limelight profile */
  public final int pipelineId;

  /** Height of the target in meters */
  public final double targetHeight;

  /** Lookup table for shooter velocity, angle, and height */
  public final VelocityAngleInterpolator lookupTable;

  /** Camera to robot transform for the targeting camera */
  public final Transform3d cameraToRobot;

  /** Indicates if this camera moves up and down with the elevator */
  public final boolean cameraOnElevator;

  /**
   * Create a limelight profile
   * @param pipelineId pipeline ID
   * @param targetHeight
   */
  private Profile(
      int pipelineId, double targetHeight, VelocityAngleInterpolator lookupTable,  Transform3d cameraToRobot,
      boolean cameraOnElevator) {
    this.pipelineId = pipelineId;
    this.targetHeight = targetHeight;
    this.lookupTable = lookupTable;
    this.cameraToRobot = cameraToRobot;
    this.cameraOnElevator = cameraOnElevator;
  }

}