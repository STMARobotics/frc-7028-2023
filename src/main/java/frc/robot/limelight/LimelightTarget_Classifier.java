package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightTarget_Classifier {

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
  public double tx;

  @JsonProperty("txp")
  public double tx_pixels;

  @JsonProperty("ty")
  public double ty;

  @JsonProperty("typ")
  public double ty_pixels;

  public LimelightTarget_Classifier() {
  }
}