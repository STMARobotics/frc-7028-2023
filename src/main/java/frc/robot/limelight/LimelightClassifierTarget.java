package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightClassifierTarget {

  /** Human-readable class name string */
  @JsonProperty("class")
  public String className;

  /** ClassID integer */
  @JsonProperty("classID")
  public double classID;

  /** Confidence of the predicition */
  @JsonProperty("conf")
  public double confidence;

  @JsonProperty("zone")
  public double zone;

  @JsonProperty("tx")
  public double targetXDegrees;

  @JsonProperty("txp")
  public double targetXPixels;

  @JsonProperty("ty")
  public double targetYDegrees;

  @JsonProperty("typ")
  public double targetYPixels;

  public LimelightClassifierTarget() {
  }
}