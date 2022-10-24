package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class HolonomicTargetCommand extends CommandBase {
  
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final PhotonCamera photoncamera;

  private final PIDController pidControllerX = new PIDController(1, 0, 0);
  private final PIDController pidControllerY = new PIDController(1.5, 0, 0);
  private final PIDController pidControllerOmega = new PIDController(.5, 0, 0);

  public HolonomicTargetCommand(DrivetrainSubsystem drivetrainSubsystem, PhotonCamera photoncamera) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.photoncamera = photoncamera;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    pidControllerX.reset();
    pidControllerY.reset();
    pidControllerOmega.reset();

    pidControllerX.setSetpoint(Units.inchesToMeters(36)); // Move forward/backwork to keep 36 inches from the target
    pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    pidControllerY.setSetpoint(0); // Move side to side to keep target centered
    pidControllerY.setTolerance(Units.inchesToMeters(2.5));

    pidControllerOmega.setSetpoint(Units.degreesToRadians(180)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));
    pidControllerOmega.enableContinuousInput(-Math.PI, Math.PI);

  }

  @Override
  public void execute() {
    var result = photoncamera.getLatestResult();
    if (result.hasTargets()) {
      var cameraToTarget = result.getBestTarget().getBestCameraToTarget();

      // X - distance from camera in meters (in meters)
      // Y - right and left of camera center (in meters)
      // Z - above and below camera center (in meters)
      // rotation Y - pitch - -90-degrees is flat on floor - rotation is positive as tilted toward camera
      //                    - visible targets in range [-90, 90]
      // rotation X - roll - 0-degrees is straight upward (or straight down) - clockwise rotation is positive
      //                  - seems to give same results if the target is upside down (maybe need to research this one)
      //                  - visible targets are in range [-180, 180]
      // rotation Z - yaw - -90-degrees is perpendicular to the camera, rotated with the right side away from camera (not visible)
      //                  - 180-degrees is straight on with the camera
      //                  - from the camera's perspective, rotation the left side of the target closer is positive
      //                  - visible targets are in range [-90, 90]

      cameraToTarget.getRotation().getAngle();
      SmartDashboard.putNumber("Target X", cameraToTarget.getX());
      SmartDashboard.putNumber("Target Y", cameraToTarget.getY());
      SmartDashboard.putNumber("Target Z", cameraToTarget.getZ());
      SmartDashboard.putNumber("Target Rotation X", Units.radiansToDegrees(cameraToTarget.getRotation().getX()));
      SmartDashboard.putNumber("Target Rotation Y", Units.radiansToDegrees(cameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("Target Rotation Z", Units.radiansToDegrees(cameraToTarget.getRotation().getZ()));

      
      // Handle distance to target
      var distanceFromTarget = cameraToTarget.getX();
      var xSpeed = pidControllerX.calculate(distanceFromTarget);
      if (pidControllerX.atSetpoint()) {
        xSpeed = 0;
      }

      // Handle alignment side-to-side
      var targetY = cameraToTarget.getY();
      var ySpeed = pidControllerY.calculate(targetY);
      if (pidControllerY.atSetpoint()) {
        ySpeed = 0;
      }

      // Handle rotation using target Yaw/Z rotation
      var targetYaw = cameraToTarget.getRotation().getZ();
      var omegaSpeed = pidControllerOmega.calculate(targetYaw);
      if (pidControllerOmega.atSetpoint()) {
        omegaSpeed = 0;
      }
      
      drivetrainSubsystem.drive(new ChassisSpeeds(-xSpeed, -ySpeed, -omegaSpeed));
    } else {
      drivetrainSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
