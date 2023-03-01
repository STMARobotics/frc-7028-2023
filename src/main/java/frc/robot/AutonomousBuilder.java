package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultWristCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.ShootConeCommand;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterProfile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutonomousBuilder {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final WristSubsystem wristSubsystem;
  private final LimelightSubsystem coneLimelightSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SwerveAutoBuilder swerveAutoBuilder;

  public AutonomousBuilder(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
      LEDSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem, WristSubsystem wristSubsystem,
      LimelightSubsystem conLimelightSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.coneLimelightSubsystem = conLimelightSubsystem;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;

    var eventMap = buildEventMap();
    swerveAutoBuilder = new SwerveAutoBuilder(
        poseEstimatorSubsystem::getCurrentPose,
        poseEstimatorSubsystem::setCurrentPose,
        DrivetrainConstants.KINEMATICS,
        new PIDConstants(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD),
        new PIDConstants(AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD),
        drivetrainSubsystem::setModuleStates,
        eventMap,
        true,
        drivetrainSubsystem
    );

    buildAutoTwoCone();

  }

  /**
   * Adds the auto chooser
   * @param dashboard dashboard
   */
  public void addDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.add("Autonomous", autoChooser).withSize(2, 1).withPosition(9, 1);
  }

  /**
   * Gets the currently selected auto command
   * @return auto command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void buildAutoTwoCone() {
    autoChooser.setDefaultOption("None", Commands.none());
    // Add all the paths in the pathplanner directory
    try (Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner"))) {
      files.filter(file -> !Files.isDirectory(file))
          .map(Path::getFileName)
          .map(Path::toString)
          .filter(fileName -> fileName.endsWith(".path"))
          .filter(fileName -> fileName.startsWith("Auto - "))
          .sorted()
          .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))
          .forEach(pathName -> autoChooser.addOption("PP: " + pathName.substring(6), buildAutoForPathGroup(pathName)));
    } catch (IOException e) {
      System.out.println("********* Failed to list PathPlanner paths. *********");
    }
  }

  private Command buildAutoForPathGroup(String pathGroupName) {
    var twoConePath = PathPlanner.loadPathGroup(pathGroupName, PathPlanner.getConstraintsFromPath(pathGroupName));
    if (twoConePath == null) {
      return Commands.print("********* Path failed to load. Not running auto: " + pathGroupName + " *********");
    }
    return swerveAutoBuilder.fullAuto(twoConePath);
  }

  private HashMap<String, Command> buildEventMap() {
    var eventMap = new HashMap<String, Command>();
    eventMap.put("shootCone1", shootCone1Event());
    eventMap.put("intakeDown", intakeDownEvent());
    eventMap.put("startIntake1", startIntakeEvent());
    eventMap.put("shootCone2", shootCone2Event());
    return eventMap;
  }

  private Command intakeDownEvent() {
    return runOnce(() -> elevatorSubsystem.moveToPosition(inchesToMeters(1.3)), elevatorSubsystem)
        .alongWith(runOnce(() -> wristSubsystem.moveToPosition(0.024), wristSubsystem));
  }
  
  private Command shootCone1Event() {
    return new JustShootCommand(inchesToMeters(1.3), 0.2, 25, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem)
        .andThen(new DefaultWristCommand(wristSubsystem)
            .alongWith(new DefaultElevatorCommand(elevatorSubsystem, shooterSubsystem::hasCone)));
  }

  private Command startIntakeEvent() {
    return new AutoIntakeCommand(inchesToMeters(1.3), 0.024, -0.1, elevatorSubsystem, wristSubsystem, shooterSubsystem).withTimeout(5.0)
        .andThen(new DefaultWristCommand(wristSubsystem)
            .alongWith(new DefaultElevatorCommand(elevatorSubsystem, shooterSubsystem::hasCone)));
  }

  private Command shootCone2Event() {
    return new ShootConeCommand(
        ShooterProfile.SCORE_CONE_TOP, LimelightProfile.SCORE_CONE_TOP, drivetrainSubsystem, elevatorSubsystem, wristSubsystem,
        shooterSubsystem, coneLimelightSubsystem, ledSubsystem).withTimeout(5.0);
  }

}
