package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class RetroLimelightTarget {

  /** Camera Pose in target space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6c_ts")
  public double[] cameraPoseTargetSpace;

  /** Robot Pose in field space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6r_fs")
  public double[] robotPoseFieldSpace;

  /** Robot Pose in target space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6r_ts")
  public double[] robotPoseTargetSpace;

  /** Target Pose in camera space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6t_cs")
  public double[] targetPoseCameraSpace;

  /** Target Pose in robot space as computed by solvepnp (x,y,z,rx,ry,rz) */
  @JsonProperty("t6t_rs")
  public double[] targetPoseRobotSpace;

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

  @JsonProperty("ts")
  public double ts;

  public RetroLimelightTarget() {
      cameraPoseTargetSpace = new double[6];
      robotPoseFieldSpace = new double[6];
      robotPoseTargetSpace = new double[6];
      targetPoseCameraSpace = new double[6];
      targetPoseRobotSpace = new double[6];
  }

}
