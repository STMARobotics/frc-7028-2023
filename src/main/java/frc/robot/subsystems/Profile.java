package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

/**
 * Limelight profiles enum
 */
public enum Profile {

  /** Middle of the Top cone vision target */
  TOP(0, Units.inchesToMeters(24.125)),
  /** Middle of the Middle cone vision target */
  MIDDLE(1, Units.inchesToMeters(43.875));

  /** ID for the Limelight profile */
  public final int pipelineId;

  /** height of the target in meters */
  public final double targetHeight;

  /**
   * Create a limelight profile
   * @param pipelineId pipeline ID
   * @param targetHeight
   */
  private Profile(int pipelineId, double targetHeight) {
    this.pipelineId = pipelineId;
    this.targetHeight = targetHeight;
  }

}