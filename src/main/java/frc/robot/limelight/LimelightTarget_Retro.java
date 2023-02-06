package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightTarget_Retro {

  /** Camera Pose in target space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6c_ts")
  public double[] cameraPose_TargetSpace;

  /** Robot Pose in field space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6r_fs")
  public double[] robotPose_FieldSpace;

  /** Robot Pose in target space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6r_ts")
  public double[] robotPose_TargetSpace;

  /** Target Pose in camera space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6t_cs")
  public double[] targetPose_CameraSpace;

  /** Target Pose in robot space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6t_rs")
  public double[] targetPose_RobotSpace;

  /** The size of the target as a percentage of the image (0-1) */
  @JsonProperty("ta")
  public double ta;

  /** X-coordinate of the center of the target in degrees. Positive-right, center-zero */
  @JsonProperty("tx")
  public double tx;

  /** X-coordinate of the center of the target in pixels. Positive-right, center-zero */
  @JsonProperty("txp")
  public double tx_pixels;

  /** Y-coordinate of the center of the target in degrees. Positive-down, center-zero */
  @JsonProperty("ty")
  public double ty;

  /** Y-coordinate of the center of the target in pixels. Positive-down, center-zero */
  @JsonProperty("typ")
  public double ty_pixels;

  @JsonProperty("ts")
  public double ts;

  public LimelightTarget_Retro() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
  }

}
