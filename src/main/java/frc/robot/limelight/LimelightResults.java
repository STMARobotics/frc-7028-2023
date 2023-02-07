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
  public RetroLimelightTarget[] RetroreflectiveTargets;

  @JsonProperty("Fiducial")
  public FiducialLimelightTarget[] FiducialTargets;

  @JsonProperty("Classifier")
  public ClassifierLimelightTarget[] classifierTargets;

  @JsonProperty("Detector")
  public DetectorLimelightTarget[] detectorTargets;

  public LimelightResults() {
      botpose = new double[6];
      botposeWPIRed = new double[6];
      botposeWPIBlue = new double[6];
      RetroreflectiveTargets = new RetroLimelightTarget[0];
      FiducialTargets = new FiducialLimelightTarget[0];
      classifierTargets = new ClassifierLimelightTarget[0];
      detectorTargets = new DetectorLimelightTarget[0];
  }
}
