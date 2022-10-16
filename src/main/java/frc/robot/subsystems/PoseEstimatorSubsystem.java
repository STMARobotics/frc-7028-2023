package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

  // Physical location of the camera on the robot, relative to the center of the
  // robot.
  private static final Transform2d CAMERA_TO_ROBOT = 
      new Transform2d(new Translation2d(Units.inchesToMeters(12.75), 0.0), new Rotation2d(0.0));

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  private static final List<Pose2d> targetPoses = Collections.unmodifiableList(List.of(
      new Pose2d(Units.inchesToMeters(84), Units.inchesToMeters(39.4375), Rotation2d.fromDegrees(180)),
      new Pose2d(Units.inchesToMeters(84), 0.0, Rotation2d.fromDegrees(180))));
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01));
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

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
    var res = photonCamera.getLatestResult();
    if (res.hasTargets()) {
      double imageCaptureTime = Timer.getFPGATimestamp() - (res.getLatencyMillis() / 1000d);

      for (PhotonTrackedTarget target : res.getTargets()) {

        var fiducialId = target.getFiducialId();
        if (fiducialId >= 0 && fiducialId < targetPoses.size()) {
          var targetPose = targetPoses.get(fiducialId);

          Transform3d camToTarget = target.getCameraToTarget();
          var transform = new Transform2d(
              camToTarget.getTranslation().toTranslation2d(),
              camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

          Pose2d camPose = targetPose.transformBy(transform.inverse());

          var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
          field2d.getObject("MyRobot" + fiducialId).setPose(visionMeasurement);
          // SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
          //   visionMeasurement.getTranslation().getX(),
          //   visionMeasurement.getTranslation().getY(),
          //   visionMeasurement.getRotation().getDegrees()));
          poseEstimator.addVisionMeasurement(visionMeasurement, imageCaptureTime);
        }
      }
          // Update pose estimator with drivetrain sensors
    }
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), drivetrainSubsystem.getGyroscopeRotation(), drivetrainSubsystem.getModuleStates());

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
