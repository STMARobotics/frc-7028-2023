package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightTarget_Detector {

  /** Human-readable class name string */
  @JsonProperty("class")
  public String className;

  /** ClassID integer */
  @JsonProperty("classID")
  public double classID;

  /** Confidence of the predicition */
  @JsonProperty("conf")
  public double confidence;

  /** The size of the target as a percentage of the image (0-1) */
  @JsonProperty("ta")
  public double targetArea;

  /** X-coordinate of the center of the target in degrees. Positive-right, center-zero */
  @JsonProperty("tx")
  public double targetXDegrees;

  /** X-coordinate of the center of the target in pixels. Positive-right, center-zero */
  @JsonProperty("txp")
  public double targetXPixels;

  /** Y-coordinate of the center of the target in degrees. Positive-down, center-zero */
  @JsonProperty("ty")
  public double targetYDegrees;

  /** Y-coordinate of the center of the target in pixels. Positive-down, center-zero */
  @JsonProperty("typ")
  public double targetYPixels;

  public LimelightTarget_Detector() {
  }
}
