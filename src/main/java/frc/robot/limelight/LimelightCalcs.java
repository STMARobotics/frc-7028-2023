package frc.robot.limelight;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Class to perform calculations from a limelight target
 */
public class LimelightCalcs {

  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private final double targetHeight;
  private final DoubleSupplier cameraHeightOffsetSupplier;

  /**
   * Constructor
   * @param cameraToRobot transform from the camera to the robot
   * @param targetHeight height of the target
   * @param cameraHeightOffsetSupplier supplier for an offset to add to the camera height. Useful for cameras
   *            mounted on things that move up and down, like elevators.
   */
  public LimelightCalcs(Transform3d cameraToRobot, double targetHeight, DoubleSupplier cameraHeightOffsetSupplier) {
    this.cameraToRobot = cameraToRobot;
    this.robotToCamera = cameraToRobot.inverse();
    this.targetHeight = targetHeight;
    this.cameraHeightOffsetSupplier = cameraHeightOffsetSupplier;
  }

  /**
   * Constructor
   * @param cameraToRobot transform from the camera to the robot
   * @param targetHeight height of the target
   */
  public LimelightCalcs(Transform3d cameraToRobot, double targetHeigt) {
    this(cameraToRobot, targetHeigt, () -> 0);
  }

  /**
   * Get distance from camera to target, along the floor
   * @param targetYDegrees Y coordinate of the target in degrees
   * @return distance from the camera to the target
   */
  protected double getCameraToTargetDistance(double targetYDegrees) {
    var cameraPitch = -cameraToRobot.getRotation().getY();
    var cameraHeight = -cameraToRobot.getZ() + cameraHeightOffsetSupplier.getAsDouble();
    return (targetHeight - cameraHeight)
        / Math.tan(cameraPitch + Units.degreesToRadians(targetYDegrees));
  }

  /**
   * Gets the robot relative translation of the target
   * @param targetYDegrees Y coordinate of the target in degrees
   * @param targetXDegrees X coordinate of the target in degrees
   * @return robot relative translaction
   */
  public Translation2d getTargetTranslation(double targetYDegrees, double targetXDegrees) {
    var targetOnCameraCoordinates = new Translation2d(
        getCameraToTargetDistance(targetYDegrees),
        Rotation2d.fromDegrees(-targetXDegrees));
    
    var targetPoseOnCameraCoordinates = new Pose2d(targetOnCameraCoordinates, new Rotation2d());

    var cameraTransform = new Transform2d(
        new Translation2d(robotToCamera.getX(), robotToCamera.getY()),
        new Rotation2d(robotToCamera.getRotation().getZ()));
    return targetPoseOnCameraCoordinates.transformBy(cameraTransform).getTranslation();
  }

  /**
   * Gets target info, relative to the robot.
   * @param targetYDegrees Y coordinate of the target in degrees
   * @param targetXDegrees X coordinate of the target in degrees
   * @return robot relative target
   */
  public VisionTargetInfo getRobotRelativeTargetInfo(double targetYDegrees, double targetXDegrees) {
    var translation = getTargetTranslation(targetYDegrees, targetXDegrees);
    var distance = translation.getDistance(new Translation2d());
    var angle = new Rotation2d(translation.getX(), translation.getY());
    return new VisionTargetInfo(translation, distance, angle);
  }

  /**
   * Gets target info, relative to the robot.
   * @param retroTarget limelight target data
   * @return robot relative target
   */
  public VisionTargetInfo getRobotRelativeTargetInfo(LimelightRetroTarget retroTarget) {
    return getRobotRelativeTargetInfo(retroTarget.targetYDegrees, retroTarget.targetYDegrees);
  }

  /**
   * Gets target info, relative to the robot.
   * @param detectorTarget limelight target data
   * @return robot relative target
   */
  public VisionTargetInfo getRobotRelativeTargetInfo(LimelightDetectorTarget detectorTarget) {
    return getRobotRelativeTargetInfo(detectorTarget.targetYDegrees, detectorTarget.targetYDegrees);
  }

}