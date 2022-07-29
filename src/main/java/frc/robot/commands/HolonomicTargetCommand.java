package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class HolonomicTargetCommand extends CommandBase {
  
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private final PIDController pidControllerY = new PIDController(1, 0, 0);
  private final PIDController pidControllerZ = new PIDController(1, 0, 0);

  public HolonomicTargetCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(drivetrainSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    pidControllerY.reset();
    pidControllerZ.reset();
  }

  @Override
  public void execute() {
    // Use target skew for side-to-side movement on Y axis
    // TODO need to understand how skew works - probably need to choose a direction based on <>-45 degrees
    var skew = limelightSubsystem.getSkew();
    var ySpeed = pidControllerY.calculate(skew);

    // Use target X for rotation around Z axis
    var targetX = limelightSubsystem.getTargetX();
    var rotation = pidControllerZ.calculate(targetX);

    drivetrainSubsystem.drive(new ChassisSpeeds(0, ySpeed, rotation));
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
