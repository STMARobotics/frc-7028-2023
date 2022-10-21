package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChaseTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS =   new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 2;
  private static final Transform2d TAG_TO_GOAL = 
      new Transform2d(new Translation2d(1.5, 0), Rotation2d.fromDegrees(180.0));

  private final PhotonCamera photonCamera;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  public ChaseTagCommand(
        PhotonCamera photonCamera, 
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget) && target.getPoseAmbiguity() <= .2 && target.getPoseAmbiguity() != -1) {
          // This is new target data, so recalculate the goal
          lastTarget = target;

          // Get the transformation from the camera to the tag (in 2d)
          var camToTarget = target.getCameraToTarget();
          var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),
            camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
          
          // Transform the robot's pose to find the tag's pose
          var cameraPose = robotPose.transformBy(CAMERA_TO_ROBOT.inverse());
          Pose2d targetPose = cameraPose.transformBy(transform);
          
          // Transform the tag's pose to set our goal
          goalPose = targetPose.transformBy(TAG_TO_GOAL);
        }

        if (null != goalPose) {
          // Drive
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }
    
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }
    
    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
