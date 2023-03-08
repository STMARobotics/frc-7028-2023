package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command to balance the charge station
 */
public class BalanceCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final boolean isReverse;
  private boolean done;

  public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem, boolean isReverse) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.isReverse = isReverse;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    done = false;
  }

  @Override
  public void execute() {
    var yAccel = drivetrainSubsystem.getGyroVelocityXYZ()[1];
    if (isReverse) {
      yAccel *= -1;
    }
    if (yAccel > 10 || done) {
      done = true;
      drivetrainSubsystem.setWheelsToX();
    } else {
      var speed = 0.5;
      if (isReverse) {
        speed *= -1;
      }
      drivetrainSubsystem.drive(new ChassisSpeeds(speed, 0.0, 0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setWheelsToX();
  }

}
