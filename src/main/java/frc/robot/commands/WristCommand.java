package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
  private final WristSubsystem wristSubsystem;
  private final DoubleSupplier speedSupplier;

  public WristCommand(WristSubsystem wristSubsystem, DoubleSupplier speedSupplier) {
    this.wristSubsystem = wristSubsystem;
    this.speedSupplier = speedSupplier;

    addRequirements(wristSubsystem);
  }

  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();
    wristSubsystem.wristUp(speed);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
  }
  
}
