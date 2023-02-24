package frc.robot.limelight;

import static frc.robot.Constants.VisionConstants.HIGH_LIMELIGHT_TO_ROBOT;
import static frc.robot.Constants.VisionConstants.LOW_LIMELIGHT_TO_ROBOT;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * Limelight profiles enum that holds info about the limelight used for various targets
 */
public enum LimelightProfile {

  /** Score a cone on a top node */
  SCORE_CONE_TOP(0, Units.inchesToMeters(44.0), LOW_LIMELIGHT_TO_ROBOT, false),

  /** Score a cone on a middle node */
  SCORE_CONE_MIDDLE(0, Units.inchesToMeters(24.125), HIGH_LIMELIGHT_TO_ROBOT, true),

  /** Score a cone in a low node */
  SCORE_CONE_LOW(0, Units.inchesToMeters(24.125), HIGH_LIMELIGHT_TO_ROBOT, true),

  /** Gamepiece on the floor */
  PICKUP_GAMEPIECE_FLOOR(1, 0.0, HIGH_LIMELIGHT_TO_ROBOT, true);

  /** ID for the Limelight profile */
  public final int pipelineId;

  /** Height of the target in meters */
  public final double targetHeight;

  /** Camera to robot transform for the targeting camera */
  public final Transform3d cameraToRobot;

  /** Indicates if this camera moves up and down with the elevator */
  public final boolean cameraOnElevator;

  /**
   * Create a limelight profile
   * @param targetHeight target height
   * @param cameraToRobot transform from the camera to the robot center
   * @param cameraOnElevator indicates if the camera is on the elevator
   */
  private LimelightProfile(
      int pipelineId, double targetHeight, Transform3d cameraToRobot, boolean cameraOnElevator) {
    this.pipelineId = pipelineId;
    this.targetHeight = targetHeight;
    this.cameraToRobot = cameraToRobot;
    this.cameraOnElevator = cameraOnElevator;
  }

}