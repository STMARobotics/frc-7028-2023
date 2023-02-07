package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command that tilts the wrist up if it is holding a cone
 */
public class DefaultWristCommand extends CommandBase {

  private final WristSubsystem wristSubsystem;
  private final BooleanSupplier raiseWrist;

  public DefaultWristCommand(WristSubsystem wristSubsystem, BooleanSupplier raiseWrist) {
    this.wristSubsystem = wristSubsystem;
    this.raiseWrist = raiseWrist;

    addRequirements(wristSubsystem);
  }

  @Override
  public void execute() {
    if (raiseWrist.getAsBoolean()) {
      wristSubsystem.moveToPosition(Math.PI/2);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
  }
}
