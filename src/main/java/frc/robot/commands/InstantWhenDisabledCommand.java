package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Utility command that is an InstantCommand that will run when the robot is disabled, to enable unit testing
 */
public class InstantWhenDisabledCommand extends InstantCommand {

  public InstantWhenDisabledCommand(Runnable runnable, Subsystem... requirements) {
    super(runnable, requirements);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
