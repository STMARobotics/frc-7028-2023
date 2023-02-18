package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command that tilts the wrist up if not doing anything else
 */
public class DefaultWristCommand extends CommandBase {

  private final WristSubsystem wristSubsystem;

  public DefaultWristCommand(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.moveToPosition(Math.PI/2);
  }

}
