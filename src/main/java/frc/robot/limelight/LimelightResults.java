package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightResults {

  /** Current pipeline index */
  @JsonProperty("pID")
  public double pipelineIndex;

  /** Targeting latency (milliseconds consumed by tracking loop this frame) */
  @JsonProperty("tl")
  public double pipelineLatency;

  @JsonProperty("tl_cap")
  public double captureLatency;

  /** Timestamp in milliseconds from boot. */
  @JsonProperty("ts")
  public double limelightTimestamp;

  @JsonProperty("ts_rio")
  public double rioTimestamp;

  /** Validity indicator. */
  @JsonProperty("v")
  @JsonFormat(shape = Shape.NUMBER)
  public boolean valid;

  /** Botpose (MegaTag): x,y,z, roll, pitch, yaw (meters, degrees) */
  @JsonProperty("botpose")
  public double[] botpose;

  /** Botpose (MegaTag, WPI Red driverstation): x,y,z, roll, pitch, yaw (meters, degrees) */
  @JsonProperty("botpose_wpired")
  public double[] botposeWPIRed;

  /** Botpose (MegaTag, WPI Blue driverstation): x,y,z, roll, pitch, yaw (meters, degrees) */
  @JsonProperty("botpose_wpiblue")
  public double[] botposeWPIBlue;

  @JsonProperty("Retro")
  public LimelightRetroTarget[] RetroreflectiveTargets;

  @JsonProperty("Fiducial")
  public LimelightFiducialTarget[] FiducialTargets;

  @JsonProperty("Classifier")
  public LimelightClassifierTarget[] classifierTargets;

  @JsonProperty("Detector")
  public LimelightDetectorTarget[] detectorTargets;

  public LimelightResults() {
      botpose = new double[6];
      botposeWPIRed = new double[6];
      botposeWPIBlue = new double[6];
      RetroreflectiveTargets = new LimelightRetroTarget[0];
      FiducialTargets = new LimelightFiducialTarget[0];
      classifierTargets = new LimelightClassifierTarget[0];
      detectorTargets = new LimelightDetectorTarget[0];
  }
}
