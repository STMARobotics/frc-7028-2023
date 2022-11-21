package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
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
      new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),
      new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0)))));
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta, s_0, ... s_n]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N7> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
  
  /**
   * Standard deviations of the encoder and gyro measurements. Increase these numbers to trust sensor readings from
   * encoders and gyros less. This matrix is in the form [theta, s_0, ... s_n], with units in radians followed by meters.
   */
  private static final Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator<N7, N7, N5> poseEstimator;

  private final Field2d field2d = new Field2d();

  private PhotonPipelineResult previousPipelineResult = null;

  public PoseEstimatorSubsystem(PhotonCamera photonCamera, DrivetrainSubsystem drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =  new SwerveDrivePoseEstimator<N7, N7, N5>(
        Nat.N7(),
        Nat.N7(),
        Nat.N5(),
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getDrivetrainState().getSwerveModulePositions(),
        new Pose2d(),
        DrivetrainSubsystem.KINEMATICS,
        stateStdDevs,
        localMeasurementStdDevs,
        visionMeasurementStdDevs);
   
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    // Update pose estimator with the best visible target
    var pipelineResult = photonCamera.getLatestResult();
    if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {
      previousPipelineResult = pipelineResult;
      double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
        var targetPose = targetPoses.get(fiducialId);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), imageCaptureTime);
      }
    }
    // Update pose estimator with drivetrain sensors
    var drivetrainState = drivetrainSubsystem.getDrivetrainState();
    poseEstimator.update(
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainState.getSwerveModuleStates(),
      drivetrainState.getSwerveModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
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
    drivetrainSubsystem.resetDriveEncoders();
    poseEstimator.resetPosition(
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainSubsystem.getDrivetrainState().getSwerveModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

}
