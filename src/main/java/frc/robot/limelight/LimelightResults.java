package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightResults {

  /** Current pipeline index */
  @JsonProperty("pID")
  public double pipelineID;

  /** Targeting latency (milliseconds consumed by tracking loop this frame) */
  @JsonProperty("tl")
  public double latency_pipeline;

  @JsonProperty("tl_cap")
  public double latency_capture;

  /** Timestamp in milliseconds from boot. */
  @JsonProperty("ts")
  public double timestamp_LIMELIGHT_publish;

  @JsonProperty("ts_rio")
  public double timestamp_RIOFPGA_capture;

  /** Validity indicator. */
  @JsonProperty("v")
  @JsonFormat(shape = Shape.NUMBER)
  public boolean valid;

  /** Botpose (MegaTag): x,y,z, roll, pitch, yaw (meters, degrees) */
  @JsonProperty("botpose")
  public double[] botpose;

  /** Botpose (MegaTag, WPI Red driverstation): x,y,z, roll, pitch, yaw (meters, degrees) */
  @JsonProperty("botpose_wpired")
  public double[] botpose_wpired;

  /** Botpose (MegaTag, WPI Blue driverstation): x,y,z, roll, pitch, yaw (meters, degrees) */
  @JsonProperty("botpose_wpiblue")
  public double[] botpose_wpiblue;

  @JsonProperty("Retro")
  public LimelightTarget_Retro[] targets_Retro;

  @JsonProperty("Fiducial")
  public LimelightTarget_Fiducial[] targets_Fiducials;

  @JsonProperty("Classifier")
  public LimelightTarget_Classifier[] targets_Classifier;

  @JsonProperty("Detector")
  public LimelightTarget_Detector[] targets_Detector;

  @JsonProperty("Barcode")
  public LimelightTarget_Barcode[] targets_Barcode;

  public LimelightResults() {
      botpose = new double[6];
      botpose_wpired = new double[6];
      botpose_wpiblue = new double[6];
      targets_Retro = new LimelightTarget_Retro[0];
      targets_Fiducials = new LimelightTarget_Fiducial[0];
      targets_Classifier = new LimelightTarget_Classifier[0];
      targets_Detector = new LimelightTarget_Detector[0];
      targets_Barcode = new LimelightTarget_Barcode[0];
  }
}
