package frc.robot.limelight;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Unit tests for LimelightCalcs. Uses some values created with a sketchup drawing.
 */
public class LimelightCalcsTest {

  private Transform3d cameraToRobot = new Transform3d(new Translation3d(-12.625, 25.6875, -13.5625), new Rotation3d());

  @Test
  public void testCameraDistanceToTarget() {
    var target = new LimelightRetroTarget();
    target.targetYDegrees = 26.7;

    var limelightCalcs = new LimelightCalcs(cameraToRobot, 53.4626);

    assertEquals(79.33, limelightCalcs.getCameraToTargetDistance(target.targetYDegrees), .005);
  }

  @Test
  public void testRobotDistanceToTarget() {
    var target = new LimelightRetroTarget();
    target.targetYDegrees = 26.7;
    target.targetXDegrees = -48.3;

    var limelightCalcs = new LimelightCalcs(cameraToRobot, 53.4626);

    var targetTranslation = limelightCalcs.getTargetTranslation(target.targetXDegrees, target.targetYDegrees);
    var distance = targetTranslation.getDistance(new Translation2d());
    assertEquals(73.4, distance, .11);
  }

  @Test
  public void testRobotAngleToTarget() {
    var target = new LimelightRetroTarget();
    target.targetYDegrees = 26.7;
    target.targetXDegrees = -48.3;

    var limelightCalcs = new LimelightCalcs(cameraToRobot, 53.4626);

    var targetPose = limelightCalcs.getTargetTranslation(target.targetXDegrees, target.targetYDegrees);
    var angle = new Rotation2d(targetPose.getX(), targetPose.getY()).getDegrees();
    assertEquals(27.1, angle, .1);
  }

  @Test
  public void testTargetOnFloor() {
    // Camera that's 3' off the floor (for simple 3,4,5 triangle)
    Transform3d highCameraToRobot = new Transform3d(new Translation3d(0.0, 0.0, -3.0), new Rotation3d());

    // Target height of zero, it's on the floor
    var limelightCalcs = new LimelightCalcs(highCameraToRobot, 0);

    // Downward 36.87-degrees, for 3,4,5 triangle
    var targetDistance = limelightCalcs.getCameraToTargetDistance(-36.87);
    assertEquals(4, targetDistance, .01);
  }

  @Test
  public void testCameraOffset() {
    // Camera that's 2' off the floor, then we'll add in 1' offset (for simple 3,4,5 triangle)
    var highCameraToRobot = new Transform3d(new Translation3d(0.0, 0.0, -2.0), new Rotation3d());

    // Target height of zero, it's on the floor
    var limelightCalcs = new LimelightCalcs(highCameraToRobot, 0, () -> 1.0);

    // Downward 36.87-degrees, for 3,4,5 triangle
    var targetDistance = limelightCalcs.getCameraToTargetDistance(-36.87);
    assertEquals(4, targetDistance, 0.01);
  }

  @Test
  public void testCameraRoll() {
    // Camera that's rotated 90-degrees counter-clockwise
    var highCameraToRobot = new Transform3d(
        new Translation3d(0.0, 0.0, -3.0), 
        new Rotation3d(Units.degreesToRadians(-90), 0.0, 0.0));

    // 3, 4, 5 triangle
    var target = new LimelightDetectorTarget();
    target.targetYDegrees = 0;
    target.targetXDegrees = -36.87;

    var limelightCalcs = new LimelightCalcs(highCameraToRobot, 0);

    var targetPose = limelightCalcs.getRobotRelativeTargetInfo(target);

    assertEquals(4, targetPose.distance, 0.01);
  }
  
}
