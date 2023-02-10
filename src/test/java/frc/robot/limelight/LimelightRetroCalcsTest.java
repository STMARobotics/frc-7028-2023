package frc.robot.limelight;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Unit tests for LimelightRetroCalcs. Uses some values created with a sketchup drawing.
 */
public class LimelightRetroCalcsTest {

  private Transform3d cameraToRobot = new Transform3d(new Translation3d(-12.625, 25.6875, -13.5625), new Rotation3d());

  @Test
  public void testCameraDistanceToTarget() {
    var target = new LimelightRetroTarget();
    target.targetYDegrees = 26.7;

    var retroCalcs = new LimelightRetroCalcs(cameraToRobot, 53.4626);

    assertEquals(79.33, retroCalcs.getCameraToTargetDistance(target), .005);
  }

  @Test
  public void testRobotDistanceToTarget() {
    var target = new LimelightRetroTarget();
    target.targetYDegrees = 26.7;
    target.targetXDegrees = -48.3;

    var retroCalcs = new LimelightRetroCalcs(cameraToRobot, 53.4626);

    var targetTranslation = retroCalcs.getTargetTranslation(target);
    var distance = targetTranslation.getDistance(new Translation2d());
    assertEquals(73.4, distance, .11);
  }

  @Test
  public void testRobotAngleToTarget() {
    var target = new LimelightRetroTarget();
    target.targetYDegrees = 26.7;
    target.targetXDegrees = -48.3;

    var retroCalcs = new LimelightRetroCalcs(cameraToRobot, 53.4626);

    var targetPose = retroCalcs.getTargetTranslation(target);
    var angle = new Rotation2d(targetPose.getX(), targetPose.getY()).getDegrees();
    assertEquals(27.1, angle, .1);
  }
  
}
