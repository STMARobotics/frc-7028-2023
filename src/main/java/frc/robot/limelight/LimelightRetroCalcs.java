package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Class to perform calculations from a limelight retroreflective target
 */
public class LimelightRetroCalcs {

  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private final double targetHeight;

  /**
   * Constructor
   * @param cameraToRobot transform from the camera to the robot
   * @param targetHeight height of the target
   */
  public LimelightRetroCalcs(Transform3d cameraToRobot, double targetHeight) {
    this.cameraToRobot = cameraToRobot;
    this.robotToCamera = cameraToRobot.inverse();
    this.targetHeight = targetHeight;
  }

  /**
   * Get distance from camera to target, along the floor
   * @param retroResults limelight target data
   * @return distance from the camera to the target
   */
  protected double getCameraToTargetDistance(LimelightRetroTarget retroResults) {
    var cameraPitch = -cameraToRobot.getRotation().getY();
    var cameraHeight = -cameraToRobot.getZ();
    return (targetHeight - cameraHeight)
        / Math.tan(cameraPitch + Units.degreesToRadians(retroResults.targetYDegrees));
  }

  /**
   * Gets the robot relative translation of the target
   * @param retroResults limelight target data
   * @return robot relative translaction
   */
  public Translation2d getTargetTranslation(LimelightRetroTarget retroResults) {
    var targetOnCameraCoordinates = new Translation2d(
        getCameraToTargetDistance(retroResults),
        Rotation2d.fromDegrees(-retroResults.targetXDegrees));
    
    var targetPoseOnCameraCoordinates = new Pose2d(targetOnCameraCoordinates, new Rotation2d());

    var cameraTransform = new Transform2d(
        new Translation2d(robotToCamera.getX(), robotToCamera.getY()),
        new Rotation2d(robotToCamera.getRotation().getZ()));
    return targetPoseOnCameraCoordinates.transformBy(cameraTransform).getTranslation();
  }

}