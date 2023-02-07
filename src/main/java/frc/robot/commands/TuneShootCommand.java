package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TuneShootCommand extends JustShootCommand {

  private final ShuffleboardTab shuffleboardTab;
  private final GenericEntry wristEntry;
  private final GenericEntry elevatorEntry;
  private final GenericEntry shooterEntry;

  public TuneShootCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {
    
    super(elevatorSubsystem, wristSubsystem, shooterSubsystem);

    shuffleboardTab = Shuffleboard.getTab("Shoot");
    wristEntry = shuffleboardTab.add("Wrist Angle", 1.127).getEntry();
    elevatorEntry = shuffleboardTab.add("Elevator Height", 0.4064).getEntry();
    shooterEntry = shuffleboardTab.add("Shooter Velocity", 34.5).getEntry();
  }

  @Override
  public void execute() {
    this.wristRadians = wristEntry.getDouble(0.0);
    this.elevatorMeters = elevatorEntry.getDouble(0.0);
    this.shooterRPS = shooterEntry.getDouble(0.0);

    super.execute();
  }

}
