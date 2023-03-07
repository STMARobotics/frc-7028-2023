package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command to balance the charge station
 */
public class BalanceBackwardsCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private boolean done;

  public BalanceBackwardsCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    done = false;
  }

  @Override
  public void execute() {
    if (drivetrainSubsystem.getGyroVelocityXYZ()[1] < -10 || done) {
      done = true;
      drivetrainSubsystem.setWheelsToX();
    } else {
      drivetrainSubsystem.drive(new ChassisSpeeds(-0.5, 0.0, 0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setWheelsToX();
  }

}
