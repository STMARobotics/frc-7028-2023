package frc.robot.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Calculated information about a retroreflective target.
 */
public class VisionTargetInfo {

  /** Translation of the target */
  public final Translation2d translation;
  
  /** Distance to the target */
  public final double distance;

  /** Angle to the target */
  public final Rotation2d angle;
  
  public VisionTargetInfo(Translation2d translation, double distance, Rotation2d angle) {
    this.translation = translation;
    this.distance = distance;
    this.angle = angle;
  }
}
