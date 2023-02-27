package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TuneShootCommand extends JustShootCommand {

  private final GenericEntry wristEntry;
  private final GenericEntry elevatorEntry;
  private final GenericEntry shooterEntry;

  /**
   * Command that reads angle, height, and velocity from the dashboard. Does not aim, just shoots with the settings.
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   */
  public TuneShootCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
    
    super(elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);

    var shuffleboardTab = Shuffleboard.getTab("Shoot");
    wristEntry = shuffleboardTab.add("Wrist Angle", 1.127).getEntry();
    elevatorEntry = shuffleboardTab.add("Elevator Height", 0.6).getEntry();
    shooterEntry = shuffleboardTab.add("Shooter Velocity", 34.5).getEntry();
  }

  @Override
  public void initialize() {
    // read shoot settings from the dashboard
    wristRadians = wristEntry.getDouble(0.0);
    elevatorMeters = elevatorEntry.getDouble(0.0);
    shooterRPS = shooterEntry.getDouble(0.0);
    super.initialize();
  }

}
