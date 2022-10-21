package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT_3D;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final DrivetrainSubsystem drivetrainSubsystem;

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
      new Pose3d(3.0, 1.166, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),
      new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0)))));
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, degreesToRadians(5));
  private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(degreesToRadians(0.01));
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, degreesToRadians(5));
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private PhotonPipelineResult previousPipelineResult = null;

  public PoseEstimatorSubsystem(PhotonCamera photonCamera, DrivetrainSubsystem drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    poseEstimator = new SwerveDrivePoseEstimator(
        drivetrainSubsystem.getGyroscopeRotation(),
        new Pose2d(),
        DrivetrainSubsystem.KINEMATICS, stateStdDevs,
        localMeasurementStdDevs, visionMeasurementStdDevs);
    
    
    tab.addString("Pose (X, Y)", this::getFomattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);
  }

  @Override
  public void periodic() {
    // Update pose estimator with visible targets
    var pipelineResult = photonCamera.getLatestResult();
    if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {
      previousPipelineResult = pipelineResult;
      double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);

      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {

        var fiducialId = target.getFiducialId();
        if (fiducialId >= 0 && fiducialId < targetPoses.size()) {
          var targetPose = targetPoses.get(fiducialId);

          Transform3d camToTarget = target.getCameraToTarget();
          // Workaround until PhotonVision changes Rotation
          camToTarget = camToTarget.plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, -Math.PI / 2)));
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

          var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT_3D);
          poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), imageCaptureTime);
        }
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainSubsystem.getModuleStates());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)", 
        Units.metersToInches(pose.getX()), 
        Units.metersToInches(pose.getY()));
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    drivetrainSubsystem.resetGyroAngle();
    poseEstimator.resetPosition(newPose, drivetrainSubsystem.getGyroscopeRotation());
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    drivetrainSubsystem.resetGyroAngle();
    poseEstimator.resetPosition(
      new Pose2d(), drivetrainSubsystem.getGyroscopeRotation());
  }

}
