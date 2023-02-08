package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.limelight.LimelightRetroCalcs;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Profile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TuneShootCommand extends JustShootCommand {

  private final LimelightSubsystem limelightSubsystem;
  private final Profile shooterProfile;
  private final GenericEntry wristEntry;
  private final GenericEntry elevatorEntry;
  private final GenericEntry shooterEntry;
  private final GenericEntry targetDistanceEntry;
  private final GenericEntry targetAngleEntry;
  private final LimelightRetroCalcs limelightCalcs;

  /**
   * Command that reads angle, height, and velocity from the dashboard. Does not aim, just shoots with the settings.
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   * @param limelightSubsystem limelight - used to put info on dashboard
   * @param shooterProfile shooter profile - used to put info on dashboard
   */
  public TuneShootCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem, Profile shooterProfile) {
    
    super(elevatorSubsystem, wristSubsystem, shooterSubsystem);

    this.limelightSubsystem = limelightSubsystem;
    this.shooterProfile = shooterProfile;
    limelightCalcs = new LimelightRetroCalcs(shooterProfile.cameraToRobot, shooterProfile.targetHeight);

    var shuffleboardTab = Shuffleboard.getTab("Shoot");
    wristEntry = shuffleboardTab.add("Wrist Angle", 1.127).getEntry();
    elevatorEntry = shuffleboardTab.add("Elevator Height", 0.4064).getEntry();
    shooterEntry = shuffleboardTab.add("Shooter Velocity", 34.5).getEntry();

    targetDistanceEntry = shuffleboardTab.add("Target Distance", 0.0).getEntry();
    targetAngleEntry = shuffleboardTab.add("Target Angle", 0.0).getEntry();
  }

  @Override
  public void initialize() {
    limelightSubsystem.enable();
    limelightSubsystem.setPipelineId(shooterProfile.pipelineId);
    super.initialize();
  }

  @Override
  public void execute() {
    // read shoot settings from the dashboard
    wristRadians = wristEntry.getDouble(0.0);
    elevatorMeters = elevatorEntry.getDouble(0.0);
    shooterRPS = shooterEntry.getDouble(0.0);
    
    // Put some target info on the dashboard
    limelightSubsystem.getLatestRetroTarget().ifPresent((target) -> {
      var targetPose = limelightCalcs.getTargetPose(target);

      targetDistanceEntry.setDouble(targetPose.getTranslation().getDistance(new Translation2d()));
      targetAngleEntry.setDouble(new Rotation2d(targetPose.getX(), targetPose.getY()).getDegrees());
    });

    super.execute();
  }

}
