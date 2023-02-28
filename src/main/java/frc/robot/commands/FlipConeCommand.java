package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class FlipConeCommand extends CommandBase {

  private final double wristAngle = 0.0;
  private final double elevatorHeight = 0.15;
  private final double rotationSpeed = 1.5;
  private final LimelightSubsystem limelightSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;

  public FlipConeCommand(LimelightSubsystem limelightSubsystem, WristSubsystem wristSubsystem,
      ElevatorSubsystem elevatorSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

    this.limelightSubsystem = limelightSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(limelightSubsystem, wristSubsystem, elevatorSubsystem, drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    wristSubsystem.moveToPosition(wristAngle);
    elevatorSubsystem.moveToPosition(elevatorHeight);

    var detectionResult = limelightSubsystem.getLatestDetectorTarget();
    detectionResult.ifPresent(detection -> {
      var targetX = detection.targetXDegrees;
      
    });

    if (Math.abs(wristSubsystem.getWristPosition() - wristAngle) < .03
      && Math.abs(elevatorSubsystem.getElevatorPosition() - elevatorHeight) < .02) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, rotationSpeed));
    } else {
      drivetrainSubsystem.stop();
    }
  }
  
}
